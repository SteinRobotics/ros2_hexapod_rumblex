#include <gtest/gtest.h>

#include <concepts>
#include <limits>
#include <numbers>
#include <type_traits>

#include "movement_request.hpp"
#include "requester/irequester.hpp"
#include "rumblex_utils/body_types.hpp"
#include "velocity.hpp"

using namespace brain;

template <class A, class B>
concept Addable = requires(A a, B b) { a + b; };

static_assert(!std::is_convertible_v<double, units::Duration>);
static_assert(!std::is_convertible_v<units::LinearVelocity, units::AngularVelocity>);
static_assert(!Addable<units::LinearVelocity, units::AngularVelocity>);
static_assert(!Addable<units::Voltage, units::Temperature>);
static_assert(!Addable<units::Temperature, units::Temperature>);
static_assert(!Addable<units::Angle, units::Length>);
static_assert(std::same_as<decltype(MovementRequest{}.duration_s), units::Duration>);
static_assert(std::same_as<decltype(RequestBase{}.minDuration), units::Duration>);

TEST(BrainUnits, EquivalentMotionUnits) {
    using mp_units::si::unit_symbols::ms;
    const units::Duration duration = 500.0 * ms;
    const units::LinearVelocity speed = 200.0 * units::mm / units::s;
    const units::Length distance = speed * duration;
    EXPECT_DOUBLE_EQ(distance.numerical_value_in(units::m), 0.1);
    const units::AngularVelocity rate = 180.0 * units::deg / units::s;
    const units::Angle angle = rate * duration;
    EXPECT_NEAR(angle.numerical_value_in(units::rad), std::numbers::pi / 2.0, 1e-12);
}

TEST(BrainUnits, CelsiusPointsAndKelvinDifferences) {
    const auto freezing = units::celsius(0.0);
    const mp_units::quantity_point<units::K, mp_units::si::absolute_zero, double> kelvin = freezing;
    EXPECT_NEAR(kelvin.quantity_from(mp_units::si::absolute_zero).numerical_value_in(units::K), 273.15,
                1e-12);
    const units::Temperature round_trip = kelvin;
    EXPECT_NEAR(units::inCelsius(round_trip), 0.0, 1e-12);
    EXPECT_DOUBLE_EQ((units::celsius(80.0) - units::celsius(60.0)).numerical_value_in(units::K), 20.0);
    EXPECT_DOUBLE_EQ(units::inCelsius(freezing + 0.2 * (units::celsius(60.0) - freezing)), 12.0);
}

TEST(BrainUnits, MessageConversionsPreservePhysicalUnits) {
    Velocity velocity;
    velocity.linear.x = 120.0 * units::mm / units::s;
    velocity.angular.z = 90.0 * units::deg / units::s;
    const auto msg = velocity.toMsg();
    EXPECT_DOUBLE_EQ(msg.linear.x, 0.12);
    EXPECT_NEAR(msg.angular.z, std::numbers::pi / 2.0, 1e-12);
    const auto restored = Velocity::fromMsg(msg);
    EXPECT_EQ(restored.linear.x, velocity.linear.x);
    EXPECT_EQ(restored.angular.z, velocity.angular.z);

    rumblex_geometry::CPose pose;
    pose.position.z = 50.0 * units::mm;
    pose.orientation.yaw = std::numbers::pi * units::rad;
    const auto pose_msg = pose.toMsg();
    EXPECT_DOUBLE_EQ(pose_msg.position.z, 0.05);
    EXPECT_NEAR(pose_msg.orientation.yaw, 180.0, 1e-12);
    EXPECT_EQ(rumblex_geometry::CPose(pose_msg), pose);
}

TEST(BrainUnits, VelocityBoundaryValidatesEveryComponentBeforeProjection) {
    geometry_msgs::msg::Twist input;
    const auto linear_limit = 100.0 * units::mm / units::s;
    const auto angular_limit = 90.0 * units::deg / units::s;
    Velocity output;
    output.linear.x = 0.05 * units::m / units::s;
    for (double* component : {&input.linear.x, &input.linear.y, &input.linear.z, &input.angular.x,
                              &input.angular.y, &input.angular.z}) {
        for (double invalid :
             {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
            *component = invalid;
            EXPECT_FALSE(limitVelocity(input, linear_limit, angular_limit, output));
            EXPECT_EQ(output.linear.x, 0.05 * units::m / units::s);
        }
        *component = 0.0;
    }
    input.linear.z = 3.0;
    input.angular.x = 2.0;
    input.angular.y = 1.0;
    EXPECT_TRUE(limitVelocity(input, linear_limit, angular_limit, output));
    EXPECT_EQ(output.toMsg(), geometry_msgs::msg::Twist());
}
