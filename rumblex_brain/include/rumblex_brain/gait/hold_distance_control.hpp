#pragma once

#include <chrono>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace brain {
// SI units at the sensor/controller boundary: metres, seconds, radians.
struct HoldDistanceSettings {
    double gain;
    double deadband;
    double max_speed;
    double max_acceleration;
};
HoldDistanceSettings loadHoldDistanceSettings(const std::shared_ptr<rclcpp::Node>& node);

class HoldDistanceLaw {
   public:
    explicit HoldDistanceLaw(HoldDistanceSettings settings) : settings_(settings) {
    }
    double update(double range, double target, double dt);

   private:
    HoldDistanceSettings settings_;
    double speed_ = 0.0;
};

// Owns only this gait's sensor subscription and measurement validation.
class RangeFeedback {
   public:
    explicit RangeFeedback(const std::shared_ptr<rclcpp::Node>& node);
    std::optional<double> range() const;
    const double freshness_s;
    const double startup_timeout_s;

   private:
    std::string body_frame_;
    std::shared_ptr<rclcpp::Node> node_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    sensor_msgs::msg::Range::ConstSharedPtr message_;
    std::chrono::steady_clock::time_point received_{};
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subscription_;
};
}  // namespace brain
