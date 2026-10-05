#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace brain {
inline double velocityLimit(const std::shared_ptr<rclcpp::Node>& node, const char* name) {
    const double value = node->has_parameter(name) ? node->get_parameter(name).as_double()
                                                   : node->declare_parameter<double>(name);
    if (!std::isfinite(value) || value <= 0.0)
        throw std::invalid_argument(std::string(name) + " must be finite and positive");
    return value;
}

// ROS planar velocities always use metres/second and radians/second.
inline bool limitVelocity(const geometry_msgs::msg::Twist& input, double linear_limit, double angular_limit,
                          geometry_msgs::msg::Twist& output) {
    for (double value : {input.linear.x, input.linear.y, input.linear.z, input.angular.x, input.angular.y,
                         input.angular.z}) {
        if (!std::isfinite(value)) return false;
    }
    output = geometry_msgs::msg::Twist();
    const double magnitude = std::hypot(input.linear.x, input.linear.y);
    const double scale = magnitude > linear_limit ? linear_limit / magnitude : 1.0;
    output.linear.x = input.linear.x * scale;
    output.linear.y = input.linear.y * scale;
    output.angular.z = std::clamp(input.angular.z, -angular_limit, angular_limit);
    return true;
}
}  // namespace brain
