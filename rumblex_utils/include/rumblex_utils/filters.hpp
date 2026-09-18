/*******************************************************************************
 * Copyright (c) 2025 Christian Stein
 ******************************************************************************/

#pragma once

#include <algorithm>
#include <limits>
#include <type_traits>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rumblex_interfaces/msg/orientation.hpp"
#include "rumblex_interfaces/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

namespace utils {

// Scalar Kalman filter
template <typename T = double>
class CKalmanFilter {
    static_assert(std::is_floating_point_v<T>, "CKalmanFilter expects a floating-point type");

   public:
    CKalmanFilter(T process_noise = T{1}, T measurement_noise = T{1}, T estimation_error = T{1},
                  T initial_state = T{})
        : process_noise_(clampNonNegative(process_noise)),
          measurement_noise_(clampNonNegative(measurement_noise)),
          estimation_error_(std::max(estimation_error, T{})),
          state_(initial_state) {
    }

    void reset(T initial_state, T estimation_error) {
        state_ = initial_state;
        estimation_error_ = std::max(estimation_error, T{});
    }

    void setProcessNoise(T process_noise) {
        process_noise_ = clampNonNegative(process_noise);
    }

    void setMeasurementNoise(T measurement_noise) {
        measurement_noise_ = clampNonNegative(measurement_noise);
    }

    [[nodiscard]] T getState() const {
        return state_;
    }
    [[nodiscard]] T getUncertainty() const {
        return estimation_error_;
    }

    T predict(T control_input = T{}, T control_gain = T{1}) {
        state_ = state_ + control_gain * control_input;
        estimation_error_ += process_noise_;
        return state_;
    }

    T update(T measurement) {
        const T innovation = measurement - state_;
        const T innovation_cov = estimation_error_ + measurement_noise_;
        const T epsilon = std::numeric_limits<T>::epsilon();

        if (innovation_cov <= epsilon) {
            return state_;
        }

        const T kalman_gain = estimation_error_ / innovation_cov;
        state_ = state_ + kalman_gain * innovation;
        estimation_error_ = (T{1} - kalman_gain) * estimation_error_;
        return state_;
    }

    T step(T measurement, T control_input = T{}, T control_gain = T{1}) {
        predict(control_input, control_gain);
        return update(measurement);
    }

   private:
    static T clampNonNegative(T value) {
        return value < T{} ? T{} : value;
    }

    T process_noise_;
    T measurement_noise_;
    T estimation_error_;
    T state_;
};

// limitation filters
template <typename T>
T limitChangeRate(const T& last_value, const T& target_value, const T& change_rate) {
    return std::clamp(target_value, last_value - change_rate, last_value + change_rate);
}

template <typename T>
T limitChangeRateUp(const T& last_value, const T& target_value, const T& change_rate) {
    if (target_value >= 0.0) {
        return std::min(target_value, last_value + change_rate);
    } else {
        return std::max(target_value, last_value - change_rate);
    }
}

template <typename T>
T limitChangeRateDown(const T& last_value, const T& target_value, const T& change_rate) {
    if (target_value >= 0.0) {
        return std::max(target_value, last_value - change_rate);
    } else {
        return std::min(target_value, last_value + change_rate);
    }
}

inline geometry_msgs::msg::Vector3 limitChangeRateVector3(const geometry_msgs::msg::Vector3& last_value,
                                                          const geometry_msgs::msg::Vector3& target_value,
                                                          double change_rate) {
    geometry_msgs::msg::Vector3 limited;
    limited.x = limitChangeRate(last_value.x, target_value.x, change_rate);
    limited.y = limitChangeRate(last_value.y, target_value.y, change_rate);
    limited.z = limitChangeRate(last_value.z, target_value.z, change_rate);
    return limited;
}

inline geometry_msgs::msg::Vector3 limitChangeRateUpVector3(const geometry_msgs::msg::Vector3& last_value,
                                                            const geometry_msgs::msg::Vector3& target_value,
                                                            double change_rate) {
    geometry_msgs::msg::Vector3 limited;
    limited.x = limitChangeRateUp(last_value.x, target_value.x, change_rate);
    limited.y = limitChangeRateUp(last_value.y, target_value.y, change_rate);
    limited.z = limitChangeRateUp(last_value.z, target_value.z, change_rate);
    return limited;
}

inline geometry_msgs::msg::Vector3 limitChangeRateDownVector3(const geometry_msgs::msg::Vector3& last_value,
                                                              const geometry_msgs::msg::Vector3& target_value,
                                                              double change_rate) {
    geometry_msgs::msg::Vector3 limited;
    limited.x = limitChangeRateDown(last_value.x, target_value.x, change_rate);
    limited.y = limitChangeRateDown(last_value.y, target_value.y, change_rate);
    limited.z = limitChangeRateDown(last_value.z, target_value.z, change_rate);
    return limited;
}

inline geometry_msgs::msg::Twist limitChangeRateTwist(const geometry_msgs::msg::Twist& last_value,
                                                      const geometry_msgs::msg::Twist& target_value,
                                                      double change_rate) {
    geometry_msgs::msg::Twist limited;
    limited.linear = limitChangeRateVector3(last_value.linear, target_value.linear, change_rate);
    limited.angular = limitChangeRateVector3(last_value.angular, target_value.angular, change_rate);
    return limited;
}

inline geometry_msgs::msg::Twist limitChangeRateUpTwist(const geometry_msgs::msg::Twist& last_value,
                                                        const geometry_msgs::msg::Twist& target_value,
                                                        double change_rate) {
    geometry_msgs::msg::Twist limited;
    limited.linear = limitChangeRateUpVector3(last_value.linear, target_value.linear, change_rate);
    limited.angular = limitChangeRateUpVector3(last_value.angular, target_value.angular, change_rate);
    return limited;
}

inline geometry_msgs::msg::Twist limitChangeRateDownTwist(const geometry_msgs::msg::Twist& last_value,
                                                          const geometry_msgs::msg::Twist& target_value,
                                                          double change_rate) {
    geometry_msgs::msg::Twist limited;
    limited.linear = limitChangeRateDownVector3(last_value.linear, target_value.linear, change_rate);
    limited.angular = limitChangeRateDownVector3(last_value.angular, target_value.angular, change_rate);
    return limited;
}

// low-pass filter
template <typename T>
T lowPassFilter(const T& last_value, const T& target_value, double alpha) {
    return alpha * target_value + (1.0 - alpha) * last_value;
}

inline geometry_msgs::msg::Vector3 lowPassFilterVector3(const geometry_msgs::msg::Vector3& last_value,
                                                        const geometry_msgs::msg::Vector3& target_value,
                                                        double alpha) {
    geometry_msgs::msg::Vector3 filtered;
    filtered.x = lowPassFilter(last_value.x, target_value.x, alpha);
    filtered.y = lowPassFilter(last_value.y, target_value.y, alpha);
    filtered.z = lowPassFilter(last_value.z, target_value.z, alpha);
    return filtered;
}

inline geometry_msgs::msg::Twist lowPassFilterTwist(const geometry_msgs::msg::Twist& last_value,
                                                    const geometry_msgs::msg::Twist& target_value,
                                                    double alpha) {
    geometry_msgs::msg::Twist filtered;
    filtered.linear = lowPassFilterVector3(last_value.linear, target_value.linear, alpha);
    filtered.angular = lowPassFilterVector3(last_value.angular, target_value.angular, alpha);
    return filtered;
}

inline rumblex_interfaces::msg::Orientation lowPassFilterOrientation(
    const rumblex_interfaces::msg::Orientation& last_value,
    const rumblex_interfaces::msg::Orientation& target_value, double alpha) {
    rumblex_interfaces::msg::Orientation filtered;
    filtered.roll = lowPassFilter(last_value.roll, target_value.roll, alpha);
    filtered.pitch = lowPassFilter(last_value.pitch, target_value.pitch, alpha);
    filtered.yaw = lowPassFilter(last_value.yaw, target_value.yaw, alpha);
    return filtered;
}

inline rumblex_interfaces::msg::Pose lowPassFilterPose(const rumblex_interfaces::msg::Pose& last_value,
                                                      const rumblex_interfaces::msg::Pose& target_value,
                                                      double alpha) {
    rumblex_interfaces::msg::Pose filtered;
    filtered.position = lowPassFilterVector3(last_value.position, target_value.position, alpha);
    filtered.orientation = lowPassFilterOrientation(last_value.orientation, target_value.orientation, alpha);
    return filtered;
}

}  // namespace utils
