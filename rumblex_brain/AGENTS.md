# mp-units style

Use `using namespace mp_units;` and `using namespace mp_units::si::unit_symbols;`
in the relevant scope, and use short names for mp-units APIs and unit symbols.
Keep the existing `units::` aliases for the robot's physical quantity types.

For angles, explicitly import `mp_units::angular::unit_symbols::rad` and
`mp_units::angular::unit_symbols::deg` as needed to preserve the distinct angular
dimension. Scope test imports inside an anonymous namespace to avoid ambiguity
with SI angle symbols.
