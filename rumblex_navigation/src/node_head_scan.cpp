/*******************************************************************************
 * Copyright (c) 2026 Christian Stein
 *
 * Sweeps the head yaw servo back-and-forth while collecting 1D range readings,
 * then publishes an aggregated sensor_msgs/LaserScan each time a full sweep
 * completes.  This converts the single-beam lidar into a narrow 2D scan that
 * RViz can display and the navigation node can use for obstacle avoidance.
 *
 * Subscribes:  /scan_1d        (sensor_msgs/Range)
 *              /joint_states   (sensor_msgs/JointState)
 * Publishes:   /scan           (sensor_msgs/LaserScan)
 *              /single_servo_request (rumblex_interfaces/ServoAngle)
 ******************************************************************************/

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

#include "rumblex_interfaces/msg/servo_angle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"

using namespace std::chrono_literals;

class NodeHeadScan : public rclcpp::Node {
   public:
    NodeHeadScan() : Node("node_head_scan") {
        // --- Parameters ---
        sweep_min_deg_ = declare_parameter("sweep_min_deg", -25.0);
        sweep_max_deg_ = declare_parameter("sweep_max_deg", 25.0);
        sweep_step_deg_ = declare_parameter("sweep_step_deg", 2.0);
        angle_tolerance_deg_ = declare_parameter("angle_tolerance_deg", 1.5);

        num_samples_ = static_cast<int>(std::round((sweep_max_deg_ - sweep_min_deg_) / sweep_step_deg_)) + 1;
        if (num_samples_ < 1) num_samples_ = 1;
        ranges_.assign(num_samples_, std::numeric_limits<float>::infinity());

        // --- Publishers ---
        scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        servo_pub_ = create_publisher<rumblex_interfaces::msg::ServoAngle>("single_servo_request", 10);

        // --- Subscribers ---
        range_sub_ = create_subscription<sensor_msgs::msg::Range>(
            "scan_1d", 10,
            [this](sensor_msgs::msg::Range::ConstSharedPtr msg) { latest_range_ = msg->range; });

        joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
                for (size_t i = 0; i < msg->name.size(); ++i) {
                    if (msg->name[i] == "head_yaw_joint") {
                        current_yaw_deg_ = msg->position[i] * 180.0 / std::numbers::pi;
                        return;
                    }
                }
            });

        // --- Sweep state ---
        current_step_ = 0;
        sweep_forward_ = true;
        target_yaw_deg_ = sweep_min_deg_;
        sweep_start_time_ = now();

        // 20 Hz control tick
        sweep_timer_ = create_wall_timer(50ms, std::bind(&NodeHeadScan::sweepTick, this));

        RCLCPP_INFO(get_logger(), "Head scan: %.1f° to %.1f° (step %.1f°, %d samples)", sweep_min_deg_,
                    sweep_max_deg_, sweep_step_deg_, num_samples_);
    }

   private:
    void sweepTick() {
        // Command head servo to current target angle
        rumblex_interfaces::msg::ServoAngle servo_msg;
        servo_msg.name = "HEAD_YAW";
        servo_msg.angle_deg = static_cast<float>(target_yaw_deg_);
        servo_pub_->publish(servo_msg);

        // Wait until the servo is close enough to the target
        if (std::abs(current_yaw_deg_ - target_yaw_deg_) > angle_tolerance_deg_) {
            return;
        }

        // Record range reading at this sweep position
        ranges_[current_step_] = latest_range_;

        // Check if this sweep direction is complete
        const bool at_end = sweep_forward_ ? (current_step_ >= num_samples_ - 1) : (current_step_ <= 0);

        if (at_end) {
            publishScan();
            sweep_forward_ = !sweep_forward_;
        }

        // Advance to next step
        current_step_ += sweep_forward_ ? 1 : -1;
        current_step_ = std::clamp(current_step_, 0, num_samples_ - 1);
        target_yaw_deg_ = sweep_min_deg_ + current_step_ * sweep_step_deg_;
    }

    void publishScan() {
        constexpr auto kDegToRad = std::numbers::pi / 180.0;

        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = now();
        scan.header.frame_id = "base_link";
        scan.angle_min = static_cast<float>(sweep_min_deg_ * kDegToRad);
        scan.angle_max = static_cast<float>(sweep_max_deg_ * kDegToRad);
        scan.angle_increment = static_cast<float>(sweep_step_deg_ * kDegToRad);
        scan.scan_time = static_cast<float>((now() - sweep_start_time_).seconds());
        scan.time_increment =
            (num_samples_ > 1) ? scan.scan_time / static_cast<float>(num_samples_ - 1) : 0.0F;
        scan.range_min = 0.05F;
        scan.range_max = 40.0F;
        scan.ranges = ranges_;

        scan_pub_->publish(scan);
        sweep_start_time_ = now();

        RCLCPP_DEBUG(get_logger(), "Published sweep scan (%.1fs, %d samples)", scan.scan_time, num_samples_);
    }

    // Parameters
    double sweep_min_deg_{};
    double sweep_max_deg_{};
    double sweep_step_deg_{};
    double angle_tolerance_deg_{};
    int num_samples_{};

    // Sweep state
    int current_step_{0};
    bool sweep_forward_{true};
    double target_yaw_deg_{0.0};
    double current_yaw_deg_{0.0};
    float latest_range_{std::numeric_limits<float>::infinity()};
    rclcpp::Time sweep_start_time_;
    std::vector<float> ranges_;

    // ROS interfaces
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<rumblex_interfaces::msg::ServoAngle>::SharedPtr servo_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::TimerBase::SharedPtr sweep_timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NodeHeadScan>());
    rclcpp::shutdown();
    return 0;
}
