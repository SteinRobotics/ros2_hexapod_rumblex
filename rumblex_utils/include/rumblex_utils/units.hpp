#pragma once

#include <mp-units/systems/angular.h>
#include <mp-units/systems/si.h>

namespace rumblex_geometry::units {

// Use a distinct angle dimension so angles cannot be mixed with scalar values.
using mp_units::angular::unit_symbols::deg;
using mp_units::angular::unit_symbols::rad;
using mp_units::si::unit_symbols::deg_C;
using mp_units::si::unit_symbols::K;
using mp_units::si::unit_symbols::m;
using mp_units::si::unit_symbols::mm;
using mp_units::si::unit_symbols::s;
using mp_units::si::unit_symbols::V;

using Length = mp_units::quantity<m, double>;
using Duration = mp_units::quantity<s, double>;
using LinearVelocity = mp_units::quantity<m / s, double>;
using AngularVelocity = mp_units::quantity<rad / s, double>;
using Voltage = mp_units::quantity<V, double>;
using Temperature = mp_units::quantity_point<deg_C, mp_units::si::zeroth_degree_Celsius, double>;

inline constexpr Temperature celsius(double value) {
    return Temperature{value * deg_C};
}

inline constexpr double inCelsius(Temperature value) {
    return value.quantity_from(mp_units::si::zeroth_degree_Celsius).numerical_value_in(deg_C);
}

using Area = mp_units::quantity<mp_units::square(m), double>;
using Angle = mp_units::quantity<deg, double>;

}  // namespace rumblex_geometry::units
