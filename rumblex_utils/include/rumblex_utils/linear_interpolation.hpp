/*
 * Copyright (c) 2025 Christian Stein
 *
 * Header-only linear interpolation utilities.
 */

#pragma once

#include <algorithm>
#include <array>
#include <type_traits>
#include <vector>

namespace rumblex_utils {

/**
 * @brief Linear interpolation utilities (header-only).
 *
 * Provides simple, constexpr-friendly linear interpolation helpers for scalar
 * arithmetic types and generic container-like types (std::vector, std::array,
 * etc.).

 * - linearInterpolate(a, b, t) : a + (b - a) * t (no clamping)
 * - linearInterpolateClamped(a, b, t) : clamps t to [0, 1] then interpolates
 * - linearInterpolateContainer(a, b, t) : element-wise interpolation for container types
 *
 * The container overloads require the container to expose:
 *  - size(), resize(size_t) and operator[]
 *  - a ::value_type that is an arithmetic type
 */

// Scalar arithmetic interpolation
template <typename T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
constexpr T linearInterpolate(T a, T b, double t) noexcept {
    return static_cast<T>(a + (b - a) * t);
}

// template <typename T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
// constexpr T linearInterpolateClamped(T a, T b, double t) noexcept {
//     if (t <= 0.0) return a;
//     if (t >= 1.0) return b;
//     return linearInterpolate<T>(a, b, t);
// }

// // Container-like interpolation (element-wise). Works for std::vector and std::array
// // Requirements: C has value_type, size(), resize(size_t) and operator[].
// template <typename C>
// C linearInterpolateContainer(const C& a, const C& b, double t) {
//     C out = a;
//     using Elem = typename C::value_type;
//     const auto n = std::min(a.size(), b.size());
//     out.resize(n);
//     for (std::size_t i = 0; i < n; ++i) {
//         out[i] = linearInterpolate<Elem>(a[i], b[i], t);
//     }
//     return out;
// }

// template <typename C>
// C linearInterpolateContainerClamped(const C& a, const C& b, double t) {
//     if (t <= 0.0) return a;
//     if (t >= 1.0) return b;
//     return linearInterpolateContainer<C>(a, b, t);
// }

}  // namespace rumblex_utils
