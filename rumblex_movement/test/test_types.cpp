#include <gtest/gtest.h>

#include <numbers>
#include <type_traits>

#include "requester/types.hpp"

using namespace rumblex_movement;
using namespace rumblex_movement::units;

static_assert(!std::is_constructible_v<CPosition, Angle, Angle, Angle>);
static_assert(!std::is_constructible_v<COrientation, Length, Length, Length>);
static_assert(!std::is_constructible_v<CLegAngles, Length, Length, Length>);
static_assert(!std::is_assignable_v<decltype(CPosition::x)&, double>);
static_assert(!std::is_assignable_v<decltype(CLegAngles::coxa)&, double>);
static_assert(!std::is_assignable_v<decltype(CLegAngles::coxa)&, Length>);
static_assert(!std::is_assignable_v<decltype(CBodyCenterOffset::psi)&, Length>);

TEST(MovementTypesTest, MembersAcceptCompatibleUnits) {
    CPosition position;
    position.x = 125.0 * mm;
    EXPECT_DOUBLE_EQ(position.x.numerical_value_in(m), 0.125);

    CLegAngles angles;
    angles.coxa = std::numbers::pi / 2.0 * rad;
    angles.femur = 45.0 * deg;
    const auto halfway = angles.linearInterpolate(CLegAngles(), 0.5);
    EXPECT_NEAR(halfway.coxa.numerical_value_in(deg), 45.0, 1e-12);
    EXPECT_DOUBLE_EQ(halfway.femur.numerical_value_in(deg), 22.5);

    CBodyCenterOffset offset;
    offset.x = 50.0 * mm;
    offset.psi = std::numbers::pi * rad;
    EXPECT_DOUBLE_EQ(offset.x.numerical_value_in(m), 0.050);
    EXPECT_NEAR(offset.psi.numerical_value_in(deg), 180.0, 1e-12);
}

TEST(MovementTypesTest, PositionToleranceHasLengthUnits) {
    const CPosition origin;
    const CPosition near(0.5 * mm, 0.0 * m, 0.0 * m);
    EXPECT_TRUE(CPosition::almostEqual(origin, near, 1.0 * mm));
    EXPECT_FALSE(CPosition::almostEqual(origin, near, 0.1 * mm));
}

TEST(MovementTypesTest, PositionAcceptsMixedLengthUnits) {
    const CPosition position(50.0 * mm, -0.1 * m, 25.0 * mm);
    EXPECT_DOUBLE_EQ(position.x.numerical_value_in(units::m), 0.050);
    EXPECT_DOUBLE_EQ(position.y.numerical_value_in(units::m), -0.1);
    EXPECT_DOUBLE_EQ(position.z.numerical_value_in(units::m), 0.025);

    const auto halfway = position.linearInterpolate(CPosition(150.0 * mm, 0.1 * m, -25.0 * mm), 0.5);
    EXPECT_NEAR(halfway.x.numerical_value_in(units::m), 0.1, 1e-12);
    EXPECT_DOUBLE_EQ(halfway.y.numerical_value_in(units::m), 0.0);
    EXPECT_DOUBLE_EQ(halfway.z.numerical_value_in(units::m), 0.0);
}

TEST(MovementTypesTest, OrientationAndLegAnglesAcceptMixedAngleUnits) {
    const COrientation orientation(std::numbers::pi / 2.0 * rad, -45.0 * deg, 0.0 * rad);
    EXPECT_NEAR(orientation.roll.numerical_value_in(units::deg), 90.0, 1e-12);
    EXPECT_DOUBLE_EQ(orientation.pitch.numerical_value_in(units::deg), -45.0);
    EXPECT_DOUBLE_EQ(orientation.yaw.numerical_value_in(units::deg), 0.0);

    const CLegAngles angles(-std::numbers::pi / 2.0 * rad, 45.0 * deg, std::numbers::pi * rad);
    EXPECT_NEAR(angles.coxa.numerical_value_in(units::deg), -90.0, 1e-12);
    EXPECT_DOUBLE_EQ(angles.femur.numerical_value_in(units::deg), 45.0);
    EXPECT_NEAR(angles.tibia.numerical_value_in(units::deg), 180.0, 1e-12);
}

TEST(MovementTypesTest, PoseConvertsLengthsAndAngles) {
    const CPose pose(100.0 * mm, 0.2 * m, -50.0 * mm, 0.0 * rad, 90.0 * deg, -std::numbers::pi * rad);
    EXPECT_DOUBLE_EQ(pose.position.x.numerical_value_in(units::m), 0.1);
    EXPECT_DOUBLE_EQ(pose.position.y.numerical_value_in(units::m), 0.2);
    EXPECT_DOUBLE_EQ(pose.position.z.numerical_value_in(units::m), -0.05);
    EXPECT_DOUBLE_EQ(pose.orientation.roll.numerical_value_in(units::deg), 0.0);
    EXPECT_DOUBLE_EQ(pose.orientation.pitch.numerical_value_in(units::deg), 90.0);
    EXPECT_NEAR(pose.orientation.yaw.numerical_value_in(units::deg), -180.0, 1e-12);
}
