/*******************************************************************************
 * Copyright (c) 2025 Christian Stein
 ******************************************************************************/

#pragma once

#include "rumblex_utils/linear_interpolation.hpp"
#include "requester/types.hpp"

namespace rumblex_utils {

inline COrientation linearInterpolate(const COrientation& from, const COrientation& to, double t) {
    // forward to member method for consistency
    return from.linearInterpolate(to, t);
}

inline CLegAngles linearInterpolate(const CLegAngles& from, const CLegAngles& to, double t) {
    return from.linearInterpolate(to, t);
}

inline CPose linearInterpolate(const CPose& from, const CPose& to, double t) {
    return from.linearInterpolate(to, t);
}

}  // namespace rumblex_utils
