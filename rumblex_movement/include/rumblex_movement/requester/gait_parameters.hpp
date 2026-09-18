#pragma once

#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace rumblex_movement {

struct Parameters {
    struct BodyRoll {
        double body_max_roll_deg{0.0};
        double body_max_pitch_deg{0.0};
    };

    struct Clap {};

    struct ContinuousPose {};

    struct HighFive {};

    struct LayDown {
        double head_max_pitch_deg{0.0};
    };

    struct LegWave {
        double leg_lift_height{0.0};
    };

    struct Ripple {
        double head_amplitude_yaw_deg{0.0};
        double factor_velocity_to_gait_cycle_time{0.0};
        double gait_step_length{0.0};
        double leg_lift_height{0.0};
    };

    struct Running {
        double head_amplitude_yaw_deg{0.0};
        double factor_velocity_to_gait_cycle_time{60.0};
        double gait_step_length{0.0};
        double leg_lift_height{0.0};
        double velocity_filter_alpha{0.01};
        double rotation_weight{0.7};
        double flight_fraction{0.15};  // fraction of half-cycle where both groups are airborne
    };

    struct Look {
        double body_max_yaw_deg{0.0};
        double head_max_yaw_deg{0.0};
    };

    struct StandUp {};

    struct SinglePose {};

    struct TestLegs {
        double coxa_delta_deg{0.0};
        double femur_delta_deg{0.0};
        double tibia_delta_deg{0.0};
    };

    struct Tripod {
        double head_amplitude_yaw_deg{0.0};
        double factor_velocity_to_gait_cycle_time{0.0};
        double gait_step_length{0.0};
        double leg_lift_height{0.0};
    };

    struct Waiting {};

    struct Wave {
        double head_amplitude_yaw_deg{0.0};
        double factor_velocity_to_gait_cycle_time{0.0};
        double gait_step_length{0.0};
        double leg_lift_height{0.0};
    };

    struct Watch {
        double body_max_yaw_deg{0.0};
        double head_max_yaw_deg{0.0};
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

    BodyRoll bodyRoll;
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
    const double body_max_roll_deg = node->declare_parameter<double>("GENERIC_BODY_MAX_ROLL");
    const double body_max_pitch_deg = node->declare_parameter<double>("GENERIC_BODY_MAX_PITCH");
    const double head_max_yaw_deg = node->declare_parameter<double>("GENERIC_HEAD_MAX_YAW");
    const double head_max_pitch_deg = node->declare_parameter<double>("GENERIC_HEAD_MAX_PITCH");

    const double leg_lift_height = node->declare_parameter<double>("GENERIC_LEG_LIFT_HEIGHT");
    const double step_length = node->declare_parameter<double>("GENERIC_STEP_LENGTH");

    // Body Roll
    params.bodyRoll.body_max_roll_deg = body_max_roll_deg;
    params.bodyRoll.body_max_pitch_deg = body_max_pitch_deg;

    // Tripod
    params.tripod.head_amplitude_yaw_deg = node->declare_parameter<double>("GAIT_TRIPOD_HEAD_MAX_YAW");
    params.tripod.factor_velocity_to_gait_cycle_time =
        node->declare_parameter<double>("GAIT_TRIPOD_FACTOR_VELOCITY_TO_CYCLE_TIME");
    params.tripod.gait_step_length = step_length;
    params.tripod.leg_lift_height = leg_lift_height;

    // Running
    params.running.head_amplitude_yaw_deg = node->declare_parameter<double>("GAIT_RUNNING_HEAD_MAX_YAW", 5.0);
    params.running.factor_velocity_to_gait_cycle_time =
        node->declare_parameter<double>("GAIT_RUNNING_FACTOR_VELOCITY_TO_CYCLE_TIME", 60.0);
    params.running.gait_step_length = step_length;
    params.running.leg_lift_height = leg_lift_height;
    params.running.velocity_filter_alpha =
        node->declare_parameter<double>("GAIT_RUNNING_VELOCITY_FILTER_ALPHA", 0.01);
    params.running.rotation_weight = node->declare_parameter<double>("GAIT_RUNNING_ROTATION_WEIGHT", 0.7);
    params.running.flight_fraction = node->declare_parameter<double>("GAIT_RUNNING_FLIGHT_FRACTION", 0.15);

    // Ripple
    params.ripple.head_amplitude_yaw_deg = node->declare_parameter<double>("GAIT_RIPPLE_HEAD_MAX_YAW", 10.0);
    params.ripple.factor_velocity_to_gait_cycle_time =
        node->declare_parameter<double>("GAIT_RIPPLE_FACTOR_VELOCITY_TO_CYCLE_TIME", 40.0);
    params.ripple.gait_step_length = step_length;
    params.ripple.leg_lift_height = leg_lift_height;

    // Wave
    params.wave.head_amplitude_yaw_deg = node->declare_parameter<double>("GAIT_WAVE_HEAD_MAX_YAW", 8.0);
    params.wave.factor_velocity_to_gait_cycle_time =
        node->declare_parameter<double>("GAIT_WAVE_FACTOR_VELOCITY_TO_CYCLE_TIME", 40.0);
    params.wave.gait_step_length = step_length;
    params.wave.leg_lift_height = leg_lift_height;

    // LayDown
    params.layDown.head_max_pitch_deg = head_max_pitch_deg;

    // Leg Wave
    params.legWave.leg_lift_height = node->declare_parameter<double>("GAIT_LEG_WAVE_LEG_LIFT_HEIGHT");

    // Look
    params.look.body_max_yaw_deg = node->declare_parameter<double>("GAIT_LOOK_BODY_MAX_YAW");
    params.look.head_max_yaw_deg = node->declare_parameter<double>("GAIT_LOOK_HEAD_MAX_YAW");

    // StandUp

    // Watch
    params.watch.body_max_yaw_deg = node->declare_parameter<double>("GAIT_WATCH_BODY_MAX_YAW");
    params.watch.head_max_yaw_deg = head_max_yaw_deg;

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
    params.testLegs.coxa_delta_deg = node->declare_parameter<double>("TESTLEGS_COXA_DELTA_DEG");
    params.testLegs.femur_delta_deg = node->declare_parameter<double>("TESTLEGS_FEMUR_DELTA_DEG");
    params.testLegs.tibia_delta_deg = node->declare_parameter<double>("TESTLEGS_TIBIA_DELTA_DEG");

    return params;
}

}  // namespace rumblex_movement
