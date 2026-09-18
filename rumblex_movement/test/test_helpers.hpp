#pragma once

#include <gtest/gtest.h>

#include <array>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "requester/gait_parameters.hpp"
#include "requester/kinematics.hpp"

namespace rumblex_movement::test_helpers {

inline std::vector<rclcpp::Parameter> defaultKinematicsParameters() {
    std::vector<rclcpp::Parameter> params = {
        rclcpp::Parameter("COXA_LENGTH", 0.050), rclcpp::Parameter("FEMUR_LENGTH", 0.063),
        rclcpp::Parameter("TIBIA_LENGTH", 0.099), rclcpp::Parameter("COXA_HEIGHT", 0.045)};

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
        {{"RightFront", 0.109, 0.068, 45.0, 0.201, 0.160, -0.050, 0.180, 0.139, 0.010},
         {"RightMid", 0.000, 0.088, 90.0, 0.000, 0.218, -0.050, 0.000, 0.188, 0.010},
         {"RightBack", -0.109, 0.068, 135.0, -0.201, 0.160, -0.050, -0.180, 0.139, 0.010},
         {"LeftFront", 0.109, -0.068, -45.0, 0.201, -0.160, -0.050, 0.180, -0.139, 0.010},
         {"LeftMid", 0.000, -0.088, -90.0, 0.000, -0.218, -0.050, 0.000, -0.188, 0.010},
         {"LeftBack", -0.109, -0.068, -135.0, -0.201, -0.160, -0.050, -0.180, -0.139, 0.010}}};

    for (const auto& leg : legs) {
        const std::string name(leg.name);
        params.emplace_back("leg_names." + name, name);
        params.emplace_back("leg_offsets." + name + ".CENTER_TO_COXA_X", leg.offset_x);
        params.emplace_back("leg_offsets." + name + ".CENTER_TO_COXA_Y", leg.offset_y);
        params.emplace_back("leg_offsets." + name + ".OFFSET_COXA_ANGLE_DEG", leg.offset_psi);

        params.emplace_back("footPositions_standing." + name + ".x", leg.standing_x);
        params.emplace_back("footPositions_standing." + name + ".y", leg.standing_y);
        params.emplace_back("footPositions_standing." + name + ".z", leg.standing_z);

        params.emplace_back("footPositions_laydown." + name + ".x", leg.laydown_x);
        params.emplace_back("footPositions_laydown." + name + ".y", leg.laydown_y);
        params.emplace_back("footPositions_laydown." + name + ".z", leg.laydown_z);
    }

    return params;
}

inline std::vector<rclcpp::Parameter> defaultGaitParameters() {
    return {rclcpp::Parameter("GENERIC_BODY_MAX_ROLL", 12.0),
            rclcpp::Parameter("GENERIC_BODY_MAX_PITCH", 12.0),
            rclcpp::Parameter("GENERIC_HEAD_MAX_YAW", 30.0),
            rclcpp::Parameter("GENERIC_HEAD_MAX_PITCH", 20.0),
            rclcpp::Parameter("GENERIC_LEG_LIFT_HEIGHT", 0.025),
            rclcpp::Parameter("GENERIC_STEP_LENGTH", 0.03),
            rclcpp::Parameter("GAIT_TRIPOD_HEAD_MAX_YAW", 15.0),
            rclcpp::Parameter("GAIT_TRIPOD_FACTOR_VELOCITY_TO_CYCLE_TIME", 40.0),
            rclcpp::Parameter("GAIT_RUNNING_FACTOR_VELOCITY_TO_CYCLE_TIME", 60.0),
            rclcpp::Parameter("GAIT_RUNNING_HEAD_MAX_YAW", 5.0),
            rclcpp::Parameter("GAIT_LEG_WAVE_LEG_LIFT_HEIGHT", 0.03),
            rclcpp::Parameter("GAIT_LOOK_BODY_MAX_YAW", 20.0),
            rclcpp::Parameter("GAIT_LOOK_HEAD_MAX_YAW", 25.0),
            rclcpp::Parameter("GAIT_WATCH_BODY_MAX_YAW", 10.0),
            rclcpp::Parameter("TESTLEGS_COXA_DELTA_DEG", 10.0),
            rclcpp::Parameter("TESTLEGS_FEMUR_DELTA_DEG", 15.0),
            rclcpp::Parameter("TESTLEGS_TIBIA_DELTA_DEG", 20.0)};
}

inline std::vector<rclcpp::Parameter> defaultRobotParameters() {
    auto params = defaultKinematicsParameters();
    const auto gait_params = defaultGaitParameters();
    params.insert(params.end(), gait_params.begin(), gait_params.end());
    return params;
}

inline Parameters makeDeclaredParameters(const std::shared_ptr<rclcpp::Node>& node) {
    return Parameters::declare(node);
}

// Reusable helper to compare CPosition
inline void expectPositionNear(const CPosition& expected, const CPosition& actual,
                               const std::string& msg = "", double tolerance = 1e-3) {
    EXPECT_NEAR(expected.x, actual.x, tolerance) << msg;
    EXPECT_NEAR(expected.y, actual.y, tolerance) << msg;
    EXPECT_NEAR(expected.z, actual.z, tolerance) << msg;
}

// Reusable helper to compare CLegAngles
inline void expectAnglesNear(const CLegAngles& expected, const CLegAngles& actual,
                             const std::string& msg = "", double tolerance = 1e-3) {
    EXPECT_NEAR(expected.coxa_deg, actual.coxa_deg, tolerance) << msg;
    EXPECT_NEAR(expected.femur_deg, actual.femur_deg, tolerance) << msg;
    EXPECT_NEAR(expected.tibia_deg, actual.tibia_deg, tolerance) << msg;
}

// Compare CPose component-wise (position + orientation)
inline void expectPoseNear(const CPose& expected, const CPose& actual, const std::string& msg = "") {
    expectPositionNear(expected.position, actual.position, msg);
    EXPECT_DOUBLE_EQ(expected.orientation.roll_deg, actual.orientation.roll_deg) << msg;
    EXPECT_DOUBLE_EQ(expected.orientation.pitch_deg, actual.orientation.pitch_deg) << msg;
    EXPECT_DOUBLE_EQ(expected.orientation.yaw_deg, actual.orientation.yaw_deg) << msg;
}

// Compare CLeg (angles + foot position)
inline void expectLegNear(const CLeg& expected, const CLeg& actual, const std::string& msg = "") {
    expectAnglesNear(expected.angles_deg_, actual.angles_deg_, msg);
    expectPositionNear(expected.foot_pos_, actual.foot_pos_, msg);
}

// Compare COrientation (roll, pitch, yaw) - for head, roll should be 0.0
inline void expectHeadNear(const COrientation& expected, const COrientation& actual,
                           const std::string& msg = "") {
    EXPECT_DOUBLE_EQ(expected.roll_deg, actual.roll_deg) << msg;
    EXPECT_DOUBLE_EQ(expected.pitch_deg, actual.pitch_deg) << msg;
    EXPECT_DOUBLE_EQ(expected.yaw_deg, actual.yaw_deg) << msg;
}

}  // namespace rumblex_movement::test_helpers

using rumblex_movement::test_helpers::expectAnglesNear;
using rumblex_movement::test_helpers::expectHeadNear;
using rumblex_movement::test_helpers::expectLegNear;
using rumblex_movement::test_helpers::expectPoseNear;
using rumblex_movement::test_helpers::expectPositionNear;
