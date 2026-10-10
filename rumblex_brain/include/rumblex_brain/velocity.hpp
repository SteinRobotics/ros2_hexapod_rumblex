#pragma once

#include <mp-units/math.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rumblex_utils/units.hpp"

namespace brain {
namespace units = rumblex_geometry::units;

// Physical velocities within the brain; ROS uses m/s and rad/s at the boundary.
struct Velocity {
    struct Linear {
        units::LinearVelocity x = 0.0 * units::m / units::s;
        units::LinearVelocity y = 0.0 * units::m / units::s;
        units::LinearVelocity z = 0.0 * units::m / units::s;
    } linear;
    struct Angular {
        units::AngularVelocity x = 0.0 * units::rad / units::s;
        units::AngularVelocity y = 0.0 * units::rad / units::s;
        units::AngularVelocity z = 0.0 * units::rad / units::s;
    } angular;

    static Velocity fromMsg(const geometry_msgs::msg::Twist& msg) {
        return {{msg.linear.x * units::m / units::s, msg.linear.y * units::m / units::s,
                 msg.linear.z * units::m / units::s},
                {msg.angular.x * units::rad / units::s, msg.angular.y * units::rad / units::s,
                 msg.angular.z * units::rad / units::s}};
    }
    geometry_msgs::msg::Twist toMsg() const {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = linear.x.numerical_value_in(units::m / units::s);
        msg.linear.y = linear.y.numerical_value_in(units::m / units::s);
        msg.linear.z = linear.z.numerical_value_in(units::m / units::s);
        msg.angular.x = angular.x.numerical_value_in(units::rad / units::s);
        msg.angular.y = angular.y.numerical_value_in(units::rad / units::s);
        msg.angular.z = angular.z.numerical_value_in(units::rad / units::s);
        return msg;
    }
};

template <auto Unit>
inline auto velocityLimit(const std::shared_ptr<rclcpp::Node>& node, const char* name) {
    const double value = node->has_parameter(name) ? node->get_parameter(name).as_double()
                                                   : node->declare_parameter<double>(name);
    if (!std::isfinite(value) || value <= 0.0)
        throw std::invalid_argument(std::string(name) + " must be finite and positive");
    return value * Unit;
}

inline bool limitVelocity(const geometry_msgs::msg::Twist& input, units::LinearVelocity linear_limit,
                          units::AngularVelocity angular_limit, Velocity& output) {
    for (double value : {input.linear.x, input.linear.y, input.linear.z, input.angular.x, input.angular.y,
                         input.angular.z}) {
        if (!std::isfinite(value)) return false;
    }
    const auto velocity = Velocity::fromMsg(input);
    output = Velocity();
    const auto magnitude = mp_units::hypot(velocity.linear.x, velocity.linear.y);
    const double scale =
        magnitude > linear_limit ? (linear_limit / magnitude).numerical_value_in(mp_units::one) : 1.0;
    output.linear.x = velocity.linear.x * scale;
    output.linear.y = velocity.linear.y * scale;
    output.angular.z = std::clamp(velocity.angular.z, -angular_limit, angular_limit);
    return true;
}
}  // namespace brain
