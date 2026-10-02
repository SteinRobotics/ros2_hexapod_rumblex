#pragma once

#include <gtest/gtest.h>

#include <array>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "requester/kinematics.hpp"

namespace rumblex_movement::test_helpers {

inline std::vector<rclcpp::Parameter> defaultKinematicsParameters() {
    std::vector<rclcpp::Parameter> params = {
        rclcpp::Parameter("coxa_length_m", 0.050), rclcpp::Parameter("femur_length_m", 0.063),
        rclcpp::Parameter("tibia_length_m", 0.099), rclcpp::Parameter("coxa_height_m", 0.045)};

    struct LegConfig {
        const char* name;
        double offset_x;
        double offset_y;
        double offset_psi;
        double standing_x;
        double standing_y;
        double standing_z;
        double laydown_x;
        double laydown_y;
        double laydown_z;
    };

    constexpr std::array<LegConfig, 6> legs = {
        {{"right_front", 0.109, 0.068, 45.0, 0.201, 0.160, -0.050, 0.180, 0.139, 0.010},
         {"right_mid", 0.000, 0.088, 90.0, 0.000, 0.218, -0.050, 0.000, 0.188, 0.010},
         {"right_back", -0.109, 0.068, 135.0, -0.201, 0.160, -0.050, -0.180, 0.139, 0.010},
         {"left_front", 0.109, -0.068, -45.0, 0.201, -0.160, -0.050, 0.180, -0.139, 0.010},
         {"left_mid", 0.000, -0.088, -90.0, 0.000, -0.218, -0.050, 0.000, -0.188, 0.010},
         {"left_back", -0.109, -0.068, -135.0, -0.201, -0.160, -0.050, -0.180, -0.139, 0.010}}};

    for (const auto& leg : legs) {
        const std::string name(leg.name);
        params.emplace_back("leg_names." + name, name);
        params.emplace_back("leg_offsets." + name + ".x_m", leg.offset_x);
        params.emplace_back("leg_offsets." + name + ".y_m", leg.offset_y);
        params.emplace_back("leg_offsets." + name + ".yaw_deg", leg.offset_psi);

        params.emplace_back("toe_positions_standing." + name + ".x", leg.standing_x);
        params.emplace_back("toe_positions_standing." + name + ".y", leg.standing_y);
        params.emplace_back("toe_positions_standing." + name + ".z", leg.standing_z);

        params.emplace_back("toe_positions_laydown." + name + ".x", leg.laydown_x);
        params.emplace_back("toe_positions_laydown." + name + ".y", leg.laydown_y);
        params.emplace_back("toe_positions_laydown." + name + ".z", leg.laydown_z);
    }

    return params;
}

inline std::vector<rclcpp::Parameter> defaultRobotParameters() {
    return defaultKinematicsParameters();
}

// Reusable helper to compare CPosition
inline void expectPositionNear(const CPosition& expected, const CPosition& actual,
                               const std::string& msg = "", double tolerance = 1e-3) {
    EXPECT_NEAR(expected.x.numerical_value_in(units::m), actual.x.numerical_value_in(units::m), tolerance)
        << msg;
    EXPECT_NEAR(expected.y.numerical_value_in(units::m), actual.y.numerical_value_in(units::m), tolerance)
        << msg;
    EXPECT_NEAR(expected.z.numerical_value_in(units::m), actual.z.numerical_value_in(units::m), tolerance)
        << msg;
}

// Reusable helper to compare CLegAngles
inline void expectAnglesNear(const CLegAngles& expected, const CLegAngles& actual,
                             const std::string& msg = "", double tolerance = 1e-3) {
    EXPECT_NEAR(expected.torso_coxa.numerical_value_in(units::deg),
                actual.torso_coxa.numerical_value_in(units::deg), tolerance)
        << msg;
    EXPECT_NEAR(expected.coxa_femur.numerical_value_in(units::deg),
                actual.coxa_femur.numerical_value_in(units::deg), tolerance)
        << msg;
    EXPECT_NEAR(expected.femur_tibia.numerical_value_in(units::deg),
                actual.femur_tibia.numerical_value_in(units::deg), tolerance)
        << msg;
}

// Compare CPose component-wise (position + orientation)
inline void expectPoseNear(const CPose& expected, const CPose& actual, const std::string& msg = "") {
    expectPositionNear(expected.position, actual.position, msg);
    EXPECT_DOUBLE_EQ(expected.orientation.roll.numerical_value_in(units::deg),
                     actual.orientation.roll.numerical_value_in(units::deg))
        << msg;
    EXPECT_DOUBLE_EQ(expected.orientation.pitch.numerical_value_in(units::deg),
                     actual.orientation.pitch.numerical_value_in(units::deg))
        << msg;
    EXPECT_DOUBLE_EQ(expected.orientation.yaw.numerical_value_in(units::deg),
                     actual.orientation.yaw.numerical_value_in(units::deg))
        << msg;
}

// Compare CLeg (angles + toe position)
inline void expectLegNear(const CLeg& expected, const CLeg& actual, const std::string& msg = "") {
    expectAnglesNear(expected.angles, actual.angles, msg);
    expectPositionNear(expected.toe_position, actual.toe_position, msg);
}

// Compare COrientation (roll, pitch, yaw) - for head, roll should be 0.0
inline void expectHeadNear(const COrientation& expected, const COrientation& actual,
                           const std::string& msg = "") {
    EXPECT_DOUBLE_EQ(expected.roll.numerical_value_in(units::deg), actual.roll.numerical_value_in(units::deg))
        << msg;
    EXPECT_DOUBLE_EQ(expected.pitch.numerical_value_in(units::deg),
                     actual.pitch.numerical_value_in(units::deg))
        << msg;
    EXPECT_DOUBLE_EQ(expected.yaw.numerical_value_in(units::deg), actual.yaw.numerical_value_in(units::deg))
        << msg;
}

}  // namespace rumblex_movement::test_helpers

using rumblex_movement::test_helpers::expectAnglesNear;
using rumblex_movement::test_helpers::expectHeadNear;
using rumblex_movement::test_helpers::expectLegNear;
using rumblex_movement::test_helpers::expectPoseNear;
using rumblex_movement::test_helpers::expectPositionNear;
