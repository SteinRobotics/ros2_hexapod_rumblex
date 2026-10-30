#pragma once

#include <chrono>
#include <cmath>
#include <stdexcept>

#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

namespace brain::feedback_detail {
inline double positive(const std::shared_ptr<rclcpp::Node>& node, const std::string& name) {
    const auto value = node->has_parameter(name) ? node->get_parameter(name).as_double()
                                                 : node->declare_parameter<double>(name);
    if (!std::isfinite(value) || value <= 0.0) throw std::invalid_argument(name + " must be positive");
    return value;
}
inline std::string textParameter(const std::shared_ptr<rclcpp::Node>& node, const std::string& name) {
    const auto value = node->has_parameter(name) ? node->get_parameter(name).as_string()
                                                 : node->declare_parameter<std::string>(name);
    if (value.empty()) throw std::invalid_argument(name + " is required");
    return value;
}
inline void validateInterval(double dt) {
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.5)
        throw std::invalid_argument("Invalid feedback update interval");
}
inline tf2::Quaternion quaternion(const geometry_msgs::msg::Quaternion& q) {
    return {q.x, q.y, q.z, q.w};
}
inline bool validQuaternion(const tf2::Quaternion& q) {
    return std::isfinite(q.length2()) && std::abs(q.length2() - 1.0) < 0.01;
}
inline bool fresh(const rclcpp::Node& node, const std_msgs::msg::Header& header,
                  std::chrono::steady_clock::time_point received, double freshness_s) {
    if (header.frame_id.empty() ||
        std::chrono::duration<double>(std::chrono::steady_clock::now() - received).count() > freshness_s ||
        (header.stamp.sec == 0 && header.stamp.nanosec == 0))
        return false;
    const double age =
        (node.now() - rclcpp::Time(header.stamp, node.get_clock()->get_clock_type())).seconds();
    return age >= -0.05 && age <= freshness_s;
}
}  // namespace brain::feedback_detail
