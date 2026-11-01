#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>

namespace brain {
enum class InterpolationType { Linear, SinSquared, Sin, Cos };

// Names describe the requested motion: Sin eases in, Cos eases out.
inline double interpolationProgress(double t, InterpolationType type) {
    t = std::clamp(t, 0.0, 1.0);
    if (t == 0.0 || t == 1.0) return t;
    const double angle = std::numbers::pi * t / 2.0;
    switch (type) {
        case InterpolationType::Linear:
            return t;
        case InterpolationType::SinSquared: {
            const double value = std::sin(angle);
            return value * value;
        }
        case InterpolationType::Sin:
            return 1.0 - std::cos(angle);
        case InterpolationType::Cos:
            return std::sin(angle);
    }
    return t;
}
}  // namespace brain
