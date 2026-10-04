#include <gtest/gtest.h>

#include <numbers>
#include <type_traits>

#include "rumblex_utils/units.hpp"

using namespace rumblex_geometry::units;

static_assert(!std::is_constructible_v<Length, decltype(1.0 * deg)>);
static_assert(!std::is_constructible_v<decltype(1.0 * rad), double>);

TEST(UnitsTest, ConvertsAngles) {
    EXPECT_NEAR((180.0 * deg).numerical_value_in(rad), std::numbers::pi, 1e-12);
    EXPECT_NEAR((-std::numbers::pi / 2.0 * rad).numerical_value_in(deg), -90.0, 1e-12);
    EXPECT_DOUBLE_EQ((0.0 * deg).numerical_value_in(rad), 0.0);
}

TEST(UnitsTest, ConvertsAndCombinesLengths) {
    const Length coxa = 50.0 * mm;
    const Length femur = 0.063 * m;
    EXPECT_DOUBLE_EQ(coxa.numerical_value_in(m), 0.050);
    EXPECT_NEAR((coxa + femur).numerical_value_in(mm), 113.0, 1e-12);
    EXPECT_NEAR((-0.050 * m).numerical_value_in(mm), -50.0, 1e-12);
}
