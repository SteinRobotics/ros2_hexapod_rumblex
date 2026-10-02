#pragma once

#include <mp-units/systems/angular.h>
#include <mp-units/systems/si.h>

namespace rumblex_geometry::units {

// Use a distinct angle dimension so angles cannot be mixed with scalar values.
using mp_units::angular::unit_symbols::deg;
using mp_units::angular::unit_symbols::rad;
using mp_units::si::unit_symbols::m;
using mp_units::si::unit_symbols::mm;

using Length = mp_units::quantity<m, double>;
using Area = mp_units::quantity<mp_units::square(m), double>;
using Angle = mp_units::quantity<deg, double>;

}  // namespace rumblex_geometry::units
