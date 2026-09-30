#pragma once

#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "units.hpp"

namespace rumblex_movement {

struct Parameters {
    struct TorsoRoll {
        units::Angle torso_max_roll = 0.0 * units::deg;
        units::Angle torso_max_pitch = 0.0 * units::deg;
    };

    struct Clap {};

    struct ContinuousPose {};

    struct HighFive {};

    struct LayDown {
        units::Angle head_max_pitch = 0.0 * units::deg;
    };

    struct LegWave {
        units::Length leg_lift_height = 0.0 * units::m;
    };

    struct Ripple {
        units::Angle head_amplitude_yaw = 0.0 * units::deg;
        double factor_velocity_to_gait_cycle_time{0.0};
        units::Length gait_step_length = 0.0 * units::m;
        units::Length leg_lift_height = 0.0 * units::m;
    };

    struct Running {
        units::Angle head_amplitude_yaw = 0.0 * units::deg;
        double factor_velocity_to_gait_cycle_time{60.0};
        units::Length gait_step_length = 0.0 * units::m;
        units::Length leg_lift_height = 0.0 * units::m;
        double velocity_filter_alpha{0.01};
        double rotation_weight{0.7};
        double flight_fraction{0.15};  // fraction of half-cycle where both groups are airborne
    };

    struct Look {
        units::Angle torso_max_yaw = 0.0 * units::deg;
        units::Angle head_max_yaw = 0.0 * units::deg;
    };

    struct StandUp {};

    struct SinglePose {};

    struct TestLegs {
        units::Angle torso_coxa_delta = 0.0 * units::deg;
        units::Angle coxa_femur_delta = 0.0 * units::deg;
        units::Angle femur_tibia_delta = 0.0 * units::deg;
    };

    struct Tripod {
        units::Angle head_amplitude_yaw = 0.0 * units::deg;
        double factor_velocity_to_gait_cycle_time{0.0};
        units::Length gait_step_length = 0.0 * units::m;
        units::Length leg_lift_height = 0.0 * units::m;
    };

    struct Waiting {};

    struct Wave {
        units::Angle head_amplitude_yaw = 0.0 * units::deg;
        double factor_velocity_to_gait_cycle_time{0.0};
        units::Length gait_step_length = 0.0 * units::m;
        units::Length leg_lift_height = 0.0 * units::m;
    };

    struct Watch {
        units::Angle torso_max_yaw = 0.0 * units::deg;
        units::Angle head_max_yaw = 0.0 * units::deg;
    };

    struct MoveCombined {
        double velocity_threshold_wave_ripple{0.3};
        double velocity_threshold_ripple_tripod{0.6};
        double hysteresis_margin{0.05};
        double transition_phase_length{M_PI};
        double velocity_filter_alpha{0.01};
        double rotation_weight{0.7};
        double max_velocity_linear{0.01};
        double max_velocity_rotation{0.01};
    };

    TorsoRoll torsoRoll;
    Clap clap;
    ContinuousPose continuousPose;
    HighFive highFive;
    LayDown layDown;
    LegWave legWave;
    Look look;
    Ripple ripple;
    Running running;
    StandUp standUp;
    SinglePose singlePose;
    TestLegs testLegs;
    Tripod tripod;
    Waiting waiting;
    Wave wave;
    Watch watch;
    MoveCombined moveCombined;

    static Parameters declare(std::shared_ptr<rclcpp::Node> node);
};

inline Parameters Parameters::declare(std::shared_ptr<rclcpp::Node> node) {
    Parameters params;

    // Generic Parameters
    const auto torso_max_roll = node->declare_parameter<double>("GENERIC_BODY_MAX_ROLL") * units::deg;
    const auto torso_max_pitch = node->declare_parameter<double>("GENERIC_BODY_MAX_PITCH") * units::deg;
    const auto head_max_yaw = node->declare_parameter<double>("GENERIC_HEAD_MAX_YAW") * units::deg;
    const auto head_max_pitch = node->declare_parameter<double>("GENERIC_HEAD_MAX_PITCH") * units::deg;

    const auto leg_lift_height = node->declare_parameter<double>("GENERIC_LEG_LIFT_HEIGHT") * units::m;
    const auto step_length = node->declare_parameter<double>("GENERIC_STEP_LENGTH") * units::m;

    // Torso Roll
    params.torsoRoll.torso_max_roll = torso_max_roll;
    params.torsoRoll.torso_max_pitch = torso_max_pitch;

    // Tripod
    params.tripod.head_amplitude_yaw =
        node->declare_parameter<double>("GAIT_TRIPOD_HEAD_MAX_YAW") * units::deg;
    params.tripod.factor_velocity_to_gait_cycle_time =
        node->declare_parameter<double>("GAIT_TRIPOD_FACTOR_VELOCITY_TO_CYCLE_TIME");
    params.tripod.gait_step_length = step_length;
    params.tripod.leg_lift_height = leg_lift_height;

    // Running
    params.running.head_amplitude_yaw =
        node->declare_parameter<double>("GAIT_RUNNING_HEAD_MAX_YAW", 5.0) * units::deg;
    params.running.factor_velocity_to_gait_cycle_time =
        node->declare_parameter<double>("GAIT_RUNNING_FACTOR_VELOCITY_TO_CYCLE_TIME", 60.0);
    params.running.gait_step_length = step_length;
    params.running.leg_lift_height = leg_lift_height;
    params.running.velocity_filter_alpha =
        node->declare_parameter<double>("GAIT_RUNNING_VELOCITY_FILTER_ALPHA", 0.01);
    params.running.rotation_weight = node->declare_parameter<double>("GAIT_RUNNING_ROTATION_WEIGHT", 0.7);
    params.running.flight_fraction = node->declare_parameter<double>("GAIT_RUNNING_FLIGHT_FRACTION", 0.15);

    // Ripple
    params.ripple.head_amplitude_yaw =
        node->declare_parameter<double>("GAIT_RIPPLE_HEAD_MAX_YAW", 10.0) * units::deg;
    params.ripple.factor_velocity_to_gait_cycle_time =
        node->declare_parameter<double>("GAIT_RIPPLE_FACTOR_VELOCITY_TO_CYCLE_TIME", 40.0);
    params.ripple.gait_step_length = step_length;
    params.ripple.leg_lift_height = leg_lift_height;

    // Wave
    params.wave.head_amplitude_yaw =
        node->declare_parameter<double>("GAIT_WAVE_HEAD_MAX_YAW", 8.0) * units::deg;
    params.wave.factor_velocity_to_gait_cycle_time =
        node->declare_parameter<double>("GAIT_WAVE_FACTOR_VELOCITY_TO_CYCLE_TIME", 40.0);
    params.wave.gait_step_length = step_length;
    params.wave.leg_lift_height = leg_lift_height;

    // LayDown
    params.layDown.head_max_pitch = head_max_pitch;

    // Leg Wave
    params.legWave.leg_lift_height =
        node->declare_parameter<double>("GAIT_LEG_WAVE_LEG_LIFT_HEIGHT") * units::m;

    // Look
    params.look.torso_max_yaw = node->declare_parameter<double>("GAIT_LOOK_BODY_MAX_YAW") * units::deg;
    params.look.head_max_yaw = node->declare_parameter<double>("GAIT_LOOK_HEAD_MAX_YAW") * units::deg;

    // StandUp

    // Watch
    params.watch.torso_max_yaw = node->declare_parameter<double>("GAIT_WATCH_BODY_MAX_YAW") * units::deg;
    params.watch.head_max_yaw = head_max_yaw;

    // MoveCombined
    params.moveCombined.velocity_threshold_wave_ripple =
        node->declare_parameter<double>("GAIT_MOVE_COMBINED_THRESHOLD_WAVE_RIPPLE", 0.3);
    params.moveCombined.velocity_threshold_ripple_tripod =
        node->declare_parameter<double>("GAIT_MOVE_COMBINED_THRESHOLD_RIPPLE_TRIPOD", 0.6);
    params.moveCombined.hysteresis_margin =
        node->declare_parameter<double>("GAIT_MOVE_COMBINED_HYSTERESIS_MARGIN", 0.05);
    params.moveCombined.transition_phase_length =
        node->declare_parameter<double>("GAIT_MOVE_COMBINED_TRANSITION_PHASE_LENGTH", M_PI);
    params.moveCombined.velocity_filter_alpha =
        node->declare_parameter<double>("GAIT_MOVE_COMBINED_VELOCITY_FILTER_ALPHA", 0.1);
    params.moveCombined.rotation_weight =
        node->declare_parameter<double>("GAIT_MOVE_COMBINED_ROTATION_WEIGHT", 0.7);
    params.moveCombined.max_velocity_linear =
        node->declare_parameter<double>("GAIT_MOVE_COMBINED_MAX_VELOCITY_LINEAR", 0.01);
    params.moveCombined.max_velocity_rotation =
        node->declare_parameter<double>("GAIT_MOVE_COMBINED_MAX_VELOCITY_ROTATION", 0.01);

    // Test Legs
    params.testLegs.torso_coxa_delta =
        node->declare_parameter<double>("TESTLEGS_COXA_DELTA_DEG") * units::deg;
    params.testLegs.coxa_femur_delta =
        node->declare_parameter<double>("TESTLEGS_FEMUR_DELTA_DEG") * units::deg;
    params.testLegs.femur_tibia_delta =
        node->declare_parameter<double>("TESTLEGS_TIBIA_DELTA_DEG") * units::deg;

    return params;
}

}  // namespace rumblex_movement
