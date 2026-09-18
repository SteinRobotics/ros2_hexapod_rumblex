/*******************************************************************************
 * Copyright (c) 2026 Christian Stein
 *
 * Simple reactive navigation for a RumbleX robot.
 *
 * Accepts 2D goal poses from RViz ("2D Nav Goal" → /goal_pose) and drives the
 * robot toward the goal while avoiding obstacles detected by the head-sweep
 * LaserScan.  Dead-reckoning odometry is maintained by integrating the
 * commanded velocities and published as both nav_msgs/Odometry and a
 * TF odom → base_link transform.
 *
 * The BNO055 IMU provides fused orientation (used for heading correction in
 * dead-reckoning odometry) and linear acceleration (used for tilt-safety).
 *
 * Subscribes:  /goal_pose    (geometry_msgs/PoseStamped)
 *              /scan         (sensor_msgs/LaserScan)
 *              /bno055/imu   (sensor_msgs/Imu)
 *              /map          (nav_msgs/OccupancyGrid)  – optional static map
 * Publishes:   /cmd_vel    (geometry_msgs/Twist)
 *              /odom       (nav_msgs/Odometry)
 *              TF: odom → base_link
 ******************************************************************************/

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
namespace {

constexpr double kDegToRad = std::numbers::pi / 180.0;
constexpr double kTwoPi = 2.0 * std::numbers::pi;

double normalizeAngle(double a) {
    while (a > std::numbers::pi) a -= kTwoPi;
    while (a < -std::numbers::pi) a += kTwoPi;
    return a;
}

}  // namespace

// ---------------------------------------------------------------------------
class NodeNavigation : public rclcpp::Node {
   public:
    NodeNavigation() : Node("node_navigation") {
        // --- Parameters ---
        goal_tolerance_ = declare_parameter("goal_tolerance_m", 0.10);
        heading_tolerance_ = declare_parameter("heading_tolerance_rad", 0.15);
        obstacle_distance_ = declare_parameter("obstacle_distance_m", 0.20);
        forward_sector_deg_ = declare_parameter("forward_sector_deg", 15.0);
        max_linear_vel_ = declare_parameter("max_linear_vel", 0.01);
        max_angular_vel_ = declare_parameter("max_angular_vel", 0.01);
        tilt_threshold_deg_ = declare_parameter("tilt_threshold_deg", 30.0);
        goal_timeout_s_ = declare_parameter("goal_timeout_s", 120.0);

        // --- Publishers ---
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // --- Subscribers ---
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
                goal_ = *msg;
                goal_start_time_ = now();
                RCLCPP_INFO(get_logger(), "Goal received: (%.2f, %.2f)", msg->pose.position.x,
                            msg->pose.position.y);
            });

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { latest_scan_ = msg; });

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "bno055/imu", 10, [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
                latest_imu_ = msg;
                // Extract yaw from orientation quaternion
                tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z,
                                  msg->orientation.w);
                imu_yaw_ = tf2::getYaw(q);
            });

        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", rclcpp::QoS(1).transient_local(),
            [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
                latest_map_ = msg;
                RCLCPP_INFO(get_logger(), "Map received (%ux%u, %.3f m/px)", msg->info.width,
                            msg->info.height, msg->info.resolution);
            });

        // --- Odometry state ---
        odom_x_ = 0.0;
        odom_y_ = 0.0;
        odom_theta_ = 0.0;
        last_time_ = now();

        // 10 Hz control loop
        control_timer_ = create_wall_timer(100ms, std::bind(&NodeNavigation::controlTick, this));

        RCLCPP_INFO(get_logger(), "Navigation ready – publish to /goal_pose to start");
    }

   private:
    // -----------------------------------------------------------------------
    // Main control loop
    // -----------------------------------------------------------------------
    void controlTick() {
        const auto current_time = now();
        const double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // 1. Update odometry (fuse dead-reckoning with IMU heading when available)
        updateOdometry(dt);
        publishOdometry();

        // 2. Tilt safety check
        if (isTilted()) {
            publishStop();
            return;
        }

        // 3. Navigate toward goal (or idle)
        if (!goal_) {
            return;
        }

        // Goal timeout
        if (goal_timeout_s_ > 0.0 && (current_time - goal_start_time_).seconds() > goal_timeout_s_) {
            RCLCPP_WARN(get_logger(), "Goal timed out after %.0f s – aborting", goal_timeout_s_);
            goal_.reset();
            publishStop();
            return;
        }

        const double dx = goal_->pose.position.x - odom_x_;
        const double dy = goal_->pose.position.y - odom_y_;
        const double distance = std::hypot(dx, dy);

        // Check arrival
        if (distance < goal_tolerance_) {
            RCLCPP_INFO(get_logger(), "Goal reached!");
            goal_.reset();
            publishStop();
            return;
        }

        // Desired heading toward the goal
        const double angle_to_goal = std::atan2(dy, dx);
        const double heading_error = normalizeAngle(angle_to_goal - odom_theta_);

        geometry_msgs::msg::Twist cmd;

        // Check for forward obstacles
        const bool obstacle_ahead = isObstacleAhead() || isMapObstacleAhead();

        if (std::abs(heading_error) > heading_tolerance_) {
            // --- Rotate in place toward goal ---
            cmd.angular.z = std::copysign(max_angular_vel_, heading_error);
        } else if (obstacle_ahead) {
            // --- Obstacle in path: rotate toward clearest direction ---
            const double clear_dir = findClearDirection();
            const double avoid_error = normalizeAngle(clear_dir - odom_theta_);
            cmd.angular.z = std::copysign(max_angular_vel_, avoid_error);
            RCLCPP_DEBUG(get_logger(), "Avoiding obstacle – rotating %.1f°", avoid_error / kDegToRad);
        } else {
            // --- Move toward goal ---
            cmd.linear.x = max_linear_vel_;
            // Proportional angular correction to keep on heading
            cmd.angular.z = std::clamp(heading_error * 2.0, -max_angular_vel_, max_angular_vel_);
        }

        last_cmd_ = cmd;
        cmd_vel_pub_->publish(cmd);
    }

    // -----------------------------------------------------------------------
    // Dead-reckoning odometry
    // -----------------------------------------------------------------------
    void updateOdometry(double dt) {
        if (dt <= 0.0 || dt > 1.0) return;  // skip bogus intervals

        // Use IMU yaw for heading when available, otherwise dead-reckon
        if (latest_imu_) {
            odom_theta_ = imu_yaw_;
        } else {
            odom_theta_ = normalizeAngle(odom_theta_ + last_cmd_.angular.z * dt);
        }
        odom_x_ += last_cmd_.linear.x * std::cos(odom_theta_) * dt;
        odom_y_ += last_cmd_.linear.x * std::sin(odom_theta_) * dt;
    }

    void publishOdometry() {
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, odom_theta_);

        // TF: odom → base_link
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = now();
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = odom_x_;
        tf.transform.translation.y = odom_y_;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(tf);

        // nav_msgs/Odometry
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = odom_x_;
        odom.pose.pose.position.y = odom_y_;
        odom.pose.pose.orientation = tf.transform.rotation;
        odom.twist.twist = last_cmd_;
        odom_pub_->publish(odom);
    }

    // -----------------------------------------------------------------------
    // Obstacle detection
    // -----------------------------------------------------------------------
    bool isObstacleAhead() const {
        if (!latest_scan_) return false;

        const auto& scan = *latest_scan_;
        const double half_sector = forward_sector_deg_ * kDegToRad;

        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
            if (std::abs(angle) <= half_sector) {
                if (std::isfinite(scan.ranges[i]) && scan.ranges[i] < obstacle_distance_) {
                    return true;
                }
            }
        }
        return false;
    }

    /// Return the absolute heading (in odom frame) of the scan sector with
    /// the largest range – i.e. the most open direction.
    double findClearDirection() const {
        if (!latest_scan_ || latest_scan_->ranges.empty()) {
            return odom_theta_;  // no data – keep current heading
        }
        const auto& scan = *latest_scan_;
        float best_range = 0.0F;
        double best_angle = 0.0;
        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            const float r = std::isfinite(scan.ranges[i]) ? scan.ranges[i] : 0.0F;
            if (r > best_range) {
                best_range = r;
                best_angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
            }
        }
        // best_angle is in base_link frame → convert to odom frame
        return normalizeAngle(odom_theta_ + best_angle);
    }

    // -----------------------------------------------------------------------
    // Map-based obstacle detection
    // -----------------------------------------------------------------------
    /// Check if the cell in front of the robot (at obstacle_distance_) is
    /// occupied according to the static map.
    bool isMapObstacleAhead() const {
        if (!latest_map_) return false;

        const auto& info = latest_map_->info;
        // Point one obstacle_distance_ ahead in the robot heading direction
        const double check_x = odom_x_ + std::cos(odom_theta_) * obstacle_distance_;
        const double check_y = odom_y_ + std::sin(odom_theta_) * obstacle_distance_;

        // Convert to grid coordinates
        const double mx = (check_x - info.origin.position.x) / info.resolution;
        const double my = (check_y - info.origin.position.y) / info.resolution;
        const auto ix = static_cast<int>(mx);
        const auto iy = static_cast<int>(my);

        if (ix < 0 || iy < 0 || static_cast<unsigned>(ix) >= info.width ||
            static_cast<unsigned>(iy) >= info.height) {
            return false;  // outside map bounds – no info
        }

        const auto cell = latest_map_->data[static_cast<size_t>(iy) * info.width + ix];
        return cell > 50;  // occupied threshold (0 = free, 100 = occupied, -1 = unknown)
    }

    // -----------------------------------------------------------------------
    // IMU tilt safety
    // -----------------------------------------------------------------------
    bool isTilted() const {
        if (!latest_imu_) return false;  // no IMU data yet

        const auto& a = latest_imu_->linear_acceleration;
        const double g_mag = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
        if (g_mag < 1.0) return false;

        const double tilt_rad = std::acos(std::clamp(a.z / g_mag, -1.0, 1.0));
        if (tilt_rad > tilt_threshold_deg_ * kDegToRad) {
            RCLCPP_WARN(get_logger(), "Tilt safety triggered: %.1f°", tilt_rad / kDegToRad);
            return true;
        }
        return false;
    }

    // -----------------------------------------------------------------------
    void publishStop() {
        geometry_msgs::msg::Twist stop;
        last_cmd_ = stop;
        cmd_vel_pub_->publish(stop);
    }

    // Parameters
    double goal_tolerance_{};
    double heading_tolerance_{};
    double obstacle_distance_{};
    double forward_sector_deg_{};
    double max_linear_vel_{};
    double max_angular_vel_{};
    double tilt_threshold_deg_{};
    double goal_timeout_s_{};

    // Odometry state
    double odom_x_{0.0};
    double odom_y_{0.0};
    double odom_theta_{0.0};
    rclcpp::Time last_time_;
    geometry_msgs::msg::Twist last_cmd_{};

    // Latest sensor data
    std::optional<geometry_msgs::msg::PoseStamped> goal_;
    rclcpp::Time goal_start_time_;
    sensor_msgs::msg::LaserScan::ConstSharedPtr latest_scan_;
    sensor_msgs::msg::Imu::ConstSharedPtr latest_imu_;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_map_;
    double imu_yaw_{0.0};

    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NodeNavigation>());
    rclcpp::shutdown();
    return 0;
}
