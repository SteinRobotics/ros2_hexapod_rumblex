#pragma once

#include <chrono>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace brain {
// SI units at the sensor/controller boundary: metres, seconds, radians.
struct LevelTorsoSettings {
    double gain;
    double deadband;
    double max_tilt;
    double max_rate;
};
struct TorsoTilt {
    double roll = 0.0;
    double pitch = 0.0;
};
LevelTorsoSettings loadLevelTorsoSettings(const std::shared_ptr<rclcpp::Node>& node);

class LevelTorsoLaw {
   public:
    explicit LevelTorsoLaw(LevelTorsoSettings settings) : settings_(settings) {
    }
    void reset(TorsoTilt initial);
    TorsoTilt update(TorsoTilt measured, double dt);

   private:
    LevelTorsoSettings settings_;
    TorsoTilt tilt_;
};

// Owns only this gait's sensor subscription and measurement validation.
class ImuFeedback {
   public:
    explicit ImuFeedback(const std::shared_ptr<rclcpp::Node>& node);
    std::optional<TorsoTilt> tilt() const;
    const double freshness_s;
    const double startup_timeout_s;

   private:
    std::string body_frame_;
    std::shared_ptr<rclcpp::Node> node_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    sensor_msgs::msg::Imu::ConstSharedPtr message_;
    std::chrono::steady_clock::time_point received_{};
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};
}  // namespace brain
