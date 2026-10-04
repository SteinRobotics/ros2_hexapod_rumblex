#pragma once

#include <algorithm>
#include <cmath>

namespace brain {
// Rest-to-rest quintic time law. Position, velocity and acceleration match at
// segment boundaries. This is the trajectory itself, not an output filter.
inline double trajectoryProgress(double t) {
    t = std::clamp(t, 0.0, 1.0);
    return t * t * t * (10.0 + t * (-15.0 + 6.0 * t));
}

// Unit-height swing with zero velocity and acceleration at lift-off/touchdown.
inline double trajectoryLift(double t) {
    t = std::clamp(t, 0.0, 1.0);
    return 64.0 * std::pow(t * (1.0 - t), 3);
}
}  // namespace brain
