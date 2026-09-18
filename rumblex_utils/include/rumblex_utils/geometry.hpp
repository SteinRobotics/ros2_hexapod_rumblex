/*******************************************************************************
 * Copyright (c) 2025 Christian Stein
 ******************************************************************************/

#pragma once

#include <cmath>

namespace utils {

constexpr double TWO_PI = 2.0 * M_PI;

inline double rad2deg(double radians) {
    return radians * 180.0 / M_PI;
}

inline double deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}

inline bool isSinValueNearZero(double angle_rad, double threshold_rad) {
    auto angle_rad_modulo_pi = std::fmod(angle_rad, M_PI);

    if (std::abs(angle_rad_modulo_pi) <= threshold_rad) {
        return true;
    }
    if (std::abs(angle_rad_modulo_pi - M_PI) <= threshold_rad) {
        return true;
    }
    return false;
}

inline bool areSinCosValuesEqual(const double angle_rad, const double angle_rad_delta) {
    auto angle_mod = std::fmod(angle_rad, TWO_PI);

    if (angle_mod - M_PI_4 >= -angle_rad_delta / 2.0 && angle_mod - M_PI_4 < angle_rad_delta / 2.0) {
        return true;
    }
    if (angle_mod - (5.0 * M_PI_4) >= -angle_rad_delta / 2.0 &&
        angle_mod - (5.0 * M_PI_4) < angle_rad_delta / 2.0) {
        return true;
    }

    return false;
}

// Checks if actual is within threshold of target
inline bool isValueNear(double target, double actual, double threshold) {
    if (actual >= target - (threshold / 2.0) && actual < target + (threshold / 2.0)) {
        return true;
    }
    return false;
}

}  // namespace utils

using utils::deg2rad;
using utils::rad2deg;
using utils::TWO_PI;