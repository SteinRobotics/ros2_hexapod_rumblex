#pragma once

#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "rumblex_utils/units.hpp"
#include "velocity.hpp"

namespace brain {
namespace units = rumblex_geometry::units;

struct Parameters {
    struct TorsoRoll {
        units::Angle torso_max_roll = 0.0 * units::deg;
        units::Angle torso_max_pitch = 0.0 * units::deg;
    };

    struct Clap {};

    struct ContinuousPose {};

    struct HighFive {};

    struct LegWave {
        units::Length leg_lift_height = 0.0 * units::m;
    };

    struct Ripple {
        units::Angle head_yaw_amplitude = 0.0 * units::deg;
        double velocity_to_phase_gain{0.0};
        units::Length gait_step_length = 0.0 * units::m;
        units::Length leg_lift_height = 0.0 * units::m;
    };

    struct Running {
        units::Angle head_yaw_amplitude = 0.0 * units::deg;
        double velocity_to_phase_gain{60.0};
        units::Length gait_step_length = 0.0 * units::m;
        units::Length leg_lift_height = 0.0 * units::m;
        double rotation_weight{0.7};
    };

    struct Look {
        units::Angle torso_max_yaw = 0.0 * units::deg;
        units::Angle head_max_yaw = 0.0 * units::deg;
    };

    struct SinglePose {
        units::Angle head_max_pitch = 0.0 * units::deg;
    };

    struct TestLegs {
        units::Angle torso_coxa_delta = 0.0 * units::deg;
        units::Angle coxa_femur_delta = 0.0 * units::deg;
        units::Angle femur_tibia_delta = 0.0 * units::deg;
    };

    struct Tripod {
        units::Angle head_yaw_amplitude = 0.0 * units::deg;
        double velocity_to_phase_gain{0.0};
        units::Length gait_step_length = 0.0 * units::m;
        units::Length leg_lift_height = 0.0 * units::m;
    };

    struct Waiting {};

    struct Wave {
        units::Angle head_yaw_amplitude = 0.0 * units::deg;
        double velocity_to_phase_gain{0.0};
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
        double rotation_weight{0.7};
        double max_linear_velocity{0.01};
        double max_angular_velocity{0.01};
    };

    TorsoRoll torso_roll;
    Clap clap;
    ContinuousPose continuous_pose;
    HighFive high_five;
    LegWave leg_wave;
    Look look;
    Ripple ripple;
    Running running;
    SinglePose single_pose;
    TestLegs test_legs;
    Tripod tripod;
    Waiting waiting;
    Wave wave;
    Watch watch;
    MoveCombined move_combined;

    static Parameters declare(std::shared_ptr<rclcpp::Node> node);
};

inline Parameters Parameters::declare(std::shared_ptr<rclcpp::Node> node) {
    Parameters params;

    // Generic Parameters
    const auto torso_max_roll =
        node->declare_parameter<double>("gait.generic.torso_max_roll_deg") * units::deg;
    const auto torso_max_pitch =
        node->declare_parameter<double>("gait.generic.torso_max_pitch_deg") * units::deg;
    const auto head_max_yaw = node->declare_parameter<double>("gait.generic.head_max_yaw_deg") * units::deg;
    const auto head_max_pitch =
        node->declare_parameter<double>("gait.generic.head_max_pitch_deg") * units::deg;

    const auto leg_lift_height = node->declare_parameter<double>("gait.generic.leg_lift_height_m") * units::m;
    const auto step_length = node->declare_parameter<double>("gait.generic.step_length_m") * units::m;

    if (!std::isfinite(step_length.numerical_value_in(units::m)) || step_length <= 0.0 * units::m)
        throw std::invalid_argument("gait.generic.step_length_m must be finite and positive");
    if (!std::isfinite(leg_lift_height.numerical_value_in(units::m)) || leg_lift_height < 0.0 * units::m)
        throw std::invalid_argument("gait.generic.leg_lift_height_m must be finite and nonnegative");

    // Torso Roll
    params.torso_roll.torso_max_roll = torso_max_roll;
    params.torso_roll.torso_max_pitch = torso_max_pitch;

    // Tripod
    params.tripod.head_yaw_amplitude =
        node->declare_parameter<double>("gait.tripod.head_max_yaw_deg") * units::deg;
    params.tripod.velocity_to_phase_gain =
        node->declare_parameter<double>("gait.tripod.velocity_to_phase_gain", 40.0);
    params.tripod.gait_step_length = step_length;
    params.tripod.leg_lift_height = leg_lift_height;

    // Running
    params.running.head_yaw_amplitude =
        node->declare_parameter<double>("gait.running.head_max_yaw_deg", 5.0) * units::deg;
    params.running.velocity_to_phase_gain =
        node->declare_parameter<double>("gait.running.velocity_to_phase_gain", 60.0);
    params.running.gait_step_length = step_length;
    params.running.leg_lift_height = leg_lift_height;
    // Legacy configuration keys remain accepted; trajectory planning does not use filters.
    node->declare_parameter<double>("gait.running.velocity_filter_alpha", 0.01);
    params.running.rotation_weight = node->declare_parameter<double>("gait.running.rotation_weight", 0.7);
    node->declare_parameter<double>("gait.running.flight_fraction", 0.15);

    // Ripple
    params.ripple.head_yaw_amplitude =
        node->declare_parameter<double>("gait.ripple.head_max_yaw_deg", 10.0) * units::deg;
    params.ripple.velocity_to_phase_gain =
        node->declare_parameter<double>("gait.ripple.velocity_to_phase_gain", 40.0);
    params.ripple.gait_step_length = step_length;
    params.ripple.leg_lift_height = leg_lift_height;

    // Wave
    params.wave.head_yaw_amplitude =
        node->declare_parameter<double>("gait.wave.head_max_yaw_deg", 8.0) * units::deg;
    params.wave.velocity_to_phase_gain =
        node->declare_parameter<double>("gait.wave.velocity_to_phase_gain", 40.0);
    params.wave.gait_step_length = step_length;
    params.wave.leg_lift_height = leg_lift_height;

    // Single Pose
    params.single_pose.head_max_pitch = head_max_pitch;

    // Leg Wave
    params.leg_wave.leg_lift_height =
        node->declare_parameter<double>("gait.leg_wave.leg_lift_height_m") * units::m;

    // Look
    params.look.torso_max_yaw = node->declare_parameter<double>("gait.look.torso_max_yaw_deg") * units::deg;
    params.look.head_max_yaw = node->declare_parameter<double>("gait.look.head_max_yaw_deg") * units::deg;

    // Watch
    params.watch.torso_max_yaw = node->declare_parameter<double>("gait.watch.torso_max_yaw_deg") * units::deg;
    params.watch.head_max_yaw = head_max_yaw;

    // MoveCombined
    params.move_combined.velocity_threshold_wave_ripple =
        node->declare_parameter<double>("gait.move_combined.velocity_threshold_wave_ripple", 0.3);
    params.move_combined.velocity_threshold_ripple_tripod =
        node->declare_parameter<double>("gait.move_combined.velocity_threshold_ripple_tripod", 0.6);
    params.move_combined.hysteresis_margin =
        node->declare_parameter<double>("gait.move_combined.hysteresis_margin", 0.05);
    node->declare_parameter<double>("gait.move_combined.transition_phase_span_rad", M_PI);
    node->declare_parameter<double>("gait.move_combined.velocity_filter_alpha", 0.1);
    params.move_combined.rotation_weight =
        node->declare_parameter<double>("gait.move_combined.rotation_weight", 0.7);
    params.move_combined.max_linear_velocity = velocityLimit(node, "max_velocity_linear");
    params.move_combined.max_angular_velocity = velocityLimit(node, "max_velocity_rotation");
    for (const auto& [name, limit] :
         {std::pair{"gait.move_combined.max_linear_velocity_m_s", params.move_combined.max_linear_velocity},
          std::pair{"gait.move_combined.max_angular_velocity_rad_s",
                    params.move_combined.max_angular_velocity}}) {
        const double alias = node->declare_parameter<double>(name, limit);
        if (!std::isfinite(alias) || std::abs(alias - limit) > 1e-12)
            throw std::invalid_argument(std::string(name) + " must match the authoritative velocity limit");
    }
    const auto& combined = params.move_combined;
    if (!(std::isfinite(combined.velocity_threshold_wave_ripple) &&
          std::isfinite(combined.velocity_threshold_ripple_tripod) &&
          std::isfinite(combined.hysteresis_margin) && combined.hysteresis_margin >= 0.0 &&
          combined.velocity_threshold_wave_ripple > combined.hysteresis_margin &&
          combined.velocity_threshold_ripple_tripod >
              combined.velocity_threshold_wave_ripple + 2.0 * combined.hysteresis_margin &&
          combined.velocity_threshold_ripple_tripod + combined.hysteresis_margin < 1.0))
        throw std::invalid_argument("Invalid combined gait thresholds or hysteresis");

    // Test Legs
    params.test_legs.torso_coxa_delta =
        node->declare_parameter<double>("gait.test_legs.torso_coxa_delta_deg") * units::deg;
    params.test_legs.coxa_femur_delta =
        node->declare_parameter<double>("gait.test_legs.coxa_femur_delta_deg") * units::deg;
    params.test_legs.femur_tibia_delta =
        node->declare_parameter<double>("gait.test_legs.femur_tibia_delta_deg") * units::deg;

    return params;
}

}  // namespace brain
