/*******************************************************************************
 * Copyright (c) 2026 Christian Stein
 ******************************************************************************/

#include "requester/gait_running.hpp"

#include <mp-units/math.h>

namespace rumblex_movement {

constexpr double kRunningTimeToWaitBeforeStopSec = 2.0;

CRunningGait::CRunningGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                           Parameters::Running& params)
    : node_(node), kinematics_(kinematics), params_(params) {
    no_velocity_timer_.stop();
    torso_old_ = CPose();
    target_positions_ = kinematics_->getStandingToePositions();
}

void CRunningGait::start(double /*duration_s*/, uint8_t /*direction*/) {
    state_ = EGaitState::Starting;
    phase_ = 0.0;
    velocity_ = geometry_msgs::msg::Twist();
    target_positions_ = kinematics_->getStandingToePositions();
}

// Running gait phase layout (one full cycle = 2π):
//
//   Group A:  [--support--|--flight--]  (phase 0 .. π)
//   Group B:  [--flight--|--support--]  (phase 0 .. π, offset by π)
//
// Within each half-cycle the leg transitions through:
//   support phase  : toe on ground, pushing backward   (0 .. π−f)
//   flight phase   : toe in air, swinging forward      (π−f .. π)
//
// where f = flight_fraction * π.
//
// Because both groups have a flight window of length f at the END of their
// respective halves, there is a brief overlap (~f) where ALL legs are airborne.
// Keeping f small (e.g. 0.15 → ~27° of the cycle) limits the unstable period.

CRunningGait::LegMotion CRunningGait::computeLegMotion(ELegIndex index, double phase) const {
    bool is_group_a = std::ranges::contains(kGroupA, index);
    double half_phase = is_group_a ? std::fmod(phase, 2.0 * M_PI) : std::fmod(phase + M_PI, 2.0 * M_PI);
    if (half_phase < 0.0) half_phase += 2.0 * M_PI;

    // Wrap into [0, π) for one half-cycle
    bool in_first_half = half_phase < M_PI;
    double local = in_first_half ? half_phase : half_phase - M_PI;

    const double f = params_.flight_fraction * M_PI;  // flight portion in radians
    const double support_end = M_PI - f;              // end of support / start of flight

    LegMotion motion;
    if (in_first_half) {
        // This group's active half: support then flight
        if (local < support_end) {
            // Support: toe on ground, slides backward
            double t = local / support_end;                            // 0..1
            motion.step = params_.gait_step_length * (1.0 - 2.0 * t);  // +step → −step
            motion.lift = 0.0 * units::m;
        } else {
            // Flight: toe in air, swings forward quickly
            double t = (local - support_end) / f;                       // 0..1
            motion.step = params_.gait_step_length * (-1.0 + 2.0 * t);  // −step → +step
            motion.lift = params_.leg_lift_height * std::sin(t * M_PI);
        }
    } else {
        // This group's passive half: toe on ground, support role
        double t = local / M_PI;  // 0..1 across the whole second half
        motion.step = params_.gait_step_length * (1.0 - 2.0 * t);
        motion.lift = 0.0 * units::m;
    }
    return motion;
}

bool CRunningGait::update(const geometry_msgs::msg::Twist& velocity, const CPose& torso,
                          const COrientation& /*head*/) {
    if (state_ == EGaitState::Stopped) {
        return false;
    }

    // Torso pose changes while stationary
    if (utils::isTwistZero(velocity) && state_ == EGaitState::Running && torso != torso_old_) {
        const auto base_toe_pos = kinematics_->getStandingToePositions();
        kinematics_->moveTorso(base_toe_pos, torso);
        torso_old_ = torso;
        return true;
    }

    // Auto-stop after idle period
    if (utils::isTwistZero(velocity) && state_ == EGaitState::Running) {
        if (!no_velocity_timer_.isRunning()) {
            no_velocity_timer_.start();
        } else if (no_velocity_timer_.haveSecondsElapsed(kRunningTimeToWaitBeforeStopSec)) {
            requestStop();
            no_velocity_timer_.stop();
        }
        return false;
    }

    no_velocity_timer_.stop();

    // Filter velocity
    if (!utils::isTwistZero(velocity)) {
        velocity_ = utils::limitChangeRateUpTwist(velocity_, velocity, params_.velocity_filter_alpha);
    }

    double linear_x = velocity_.linear.x;
    double linear_y = velocity_.linear.y;
    double angular_z = velocity_.angular.z;

    double combined_mag = std::sqrt((linear_x * linear_x) + (linear_y * linear_y) +
                                    (params_.rotation_weight * angular_z * angular_z));
    if (combined_mag < 1e-6) return false;

    double norm_x = linear_x / combined_mag;
    double norm_y = linear_y / combined_mag;
    double norm_rot = angular_z / combined_mag;

    double delta_phase = params_.velocity_to_phase_gain * combined_mag;
    phase_ += delta_phase;
    phase_ = std::fmod(phase_, 2.0 * M_PI);

    // State transitions
    if (state_ == EGaitState::Starting && phase_ > M_PI / 6.0) {
        state_ = EGaitState::Running;
    }
    if (state_ == EGaitState::StopPending && utils::isSinValueNearZero(phase_, delta_phase)) {
        RCLCPP_INFO(node_->get_logger(), "CRunningGait: Transitioning to Stopped, phase_: %.2f", phase_);
        phase_ = 0.0;
        state_ = EGaitState::Stopped;
        kinematics_->moveTorso(kinematics_->getStandingToePositions(), torso);
        kinematics_->setHeadOrientation(COrientation(0.0, 0.0, 0.0));
        return true;
    }

    const auto standing_positions = kinematics_->getStandingToePositions();

    for (auto& [index, leg] : kinematics_->getLegs()) {
        const auto base_toe_pos = standing_positions.at(index);
        auto motion = computeLegMotion(index, phase_);

        // Linear displacement
        const auto delta_x = norm_x * motion.step;
        const auto delta_y = norm_y * motion.step;

        // Rotational displacement
        const auto leg_vec_x = base_toe_pos.x;
        const auto leg_vec_y = base_toe_pos.y;
        const auto len = mp_units::hypot(leg_vec_x, leg_vec_y);

        units::Length rot_x = 0.0 * units::m;
        units::Length rot_y = 0.0 * units::m;
        if (len > 1e-6 * units::m) {
            const auto dir_x = -leg_vec_y / len;
            const auto dir_y = leg_vec_x / len;
            rot_x = dir_x * motion.step * norm_rot;
            rot_y = dir_y * motion.step * norm_rot;
        }

        CPosition new_pos;
        new_pos.x = base_toe_pos.x + delta_x + rot_x;
        new_pos.y = base_toe_pos.y + delta_y + rot_y;
        new_pos.z = base_toe_pos.z + motion.lift;

        target_positions_[index] = new_pos;
    }

    kinematics_->moveTorso(target_positions_, torso);

    // Head movement
    const auto head_yaw = params_.head_yaw_amplitude * std::sin(phase_);
    COrientation head_request;
    head_request.yaw = head_yaw;
    kinematics_->setHeadOrientation(head_request);
    return true;
}

void CRunningGait::requestStop() {
    if (state_ == EGaitState::Running) {
        state_ = EGaitState::StopPending;
    }
}

void CRunningGait::cancelStop() {
    if (state_ == EGaitState::StopPending) {
        state_ = EGaitState::Running;
    }
}

}  // namespace rumblex_movement
