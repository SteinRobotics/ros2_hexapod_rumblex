/*******************************************************************************
 * Copyright (c) 2025 Christian Stein
 ******************************************************************************/

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>

namespace utils {

constexpr double EPS = 1e-6;

inline bool isTwistZero(const geometry_msgs::msg::Twist& t) {
    const std::array<double, 6> vals{t.linear.x,  t.linear.y,  t.linear.z,
                                     t.angular.x, t.angular.y, t.angular.z};
    return std::all_of(vals.begin(), vals.end(),
                       [](double v) { return std::isfinite(v) && std::abs(v) <= EPS; });
}

inline bool hasChanged(const geometry_msgs::msg::Twist& t1, const geometry_msgs::msg::Twist& t2,
                       double threshold = EPS) {
    const std::array<double, 6> diffs{t1.linear.x - t2.linear.x,   t1.linear.y - t2.linear.y,
                                      t1.linear.z - t2.linear.z,   t1.angular.x - t2.angular.x,
                                      t1.angular.y - t2.angular.y, t1.angular.z - t2.angular.z};
    return std::any_of(diffs.begin(), diffs.end(),
                       [threshold](double d) { return !std::isfinite(d) || std::abs(d) > threshold; });
}

}  // namespace utils

using utils::hasChanged;
using utils::isTwistZero;