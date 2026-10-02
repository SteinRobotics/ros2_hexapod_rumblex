#include "gait/gait_move_combined.hpp"

#include <mp-units/math.h>

namespace brain {

constexpr double kCombinedTimeToWaitBeforeStopSec = 3.0;

CMoveCombinedGait::CMoveCombinedGait(std::shared_ptr<rclcpp::Node> node,
                                     std::shared_ptr<CPoseModel> kinematics, Parameters::Wave& wave_params,
                                     Parameters::Ripple& ripple_params, Parameters::Tripod& tripod_params,
                                     Parameters::MoveCombined& combined_params)
    : node_(node),
      kinematics_(kinematics),
      wave_params_(wave_params),
      ripple_params_(ripple_params),
      tripod_params_(tripod_params),
      combined_params_(combined_params) {
    no_velocity_timer_.stop();
    torso_old_ = CPose();
    target_positions_ = kinematics_->getStandingToePositions();
    blend_start_positions_ = target_positions_;
}

void CMoveCombinedGait::start(double /*duration_s*/, uint8_t /*direction*/) {
    state_ = EGaitState::Starting;
    no_velocity_timer_.stop();
    phase_ = 0.0;
    velocity_ = geometry_msgs::msg::Twist();
    active_gait_type_ = EMoveCombinedGaitType::Wave;
    is_blending_ = false;
    blend_alpha_ = 1.0;
    blend_start_head_yaw_ = 0.0 * units::deg;
    target_positions_ = kinematics_->getStandingToePositions();
    blend_start_positions_ = target_positions_;
}

EMoveCombinedGaitType CMoveCombinedGait::selectGait(double combined_mag) const {
    const double hyst = combined_params_.hysteresis_margin;
    const double thresh_wr = combined_params_.velocity_threshold_wave_ripple;
    const double thresh_rt = combined_params_.velocity_threshold_ripple_tripod;

    switch (active_gait_type_) {
        case EMoveCombinedGaitType::Wave:
            if (combined_mag > thresh_wr + hyst) return EMoveCombinedGaitType::Ripple;
            return EMoveCombinedGaitType::Wave;
        case EMoveCombinedGaitType::Ripple:
            if (combined_mag < thresh_wr - hyst) return EMoveCombinedGaitType::Wave;
            if (combined_mag > thresh_rt + hyst) return EMoveCombinedGaitType::Tripod;
            return EMoveCombinedGaitType::Ripple;
        case EMoveCombinedGaitType::Tripod:
            if (combined_mag < thresh_rt - hyst) return EMoveCombinedGaitType::Ripple;
            return EMoveCombinedGaitType::Tripod;
    }
    return active_gait_type_;
}

CMoveCombinedGait::LegMotion CMoveCombinedGait::computeWaveLegMotion(ELegIndex index, double phase) const {
    double phase_offset = 0.0;
    for (const auto& info : kWaveLegPhases) {
        if (info.index == index) {
            phase_offset = info.phase_offset;
            break;
        }
    }

    double local_phase = std::fmod(phase + phase_offset, 2.0 * M_PI);
    if (local_phase < 0.0) local_phase += 2.0 * M_PI;

    LegMotion motion;
    if (local_phase < kWaveTransferPhase) {
        double t = local_phase / kWaveTransferPhase;
        motion.step = -std::cos(t * M_PI) * wave_params_.gait_step_length;
        motion.lift = std::sin(t * M_PI) * wave_params_.leg_lift_height;
    } else {
        double t = (local_phase - kWaveTransferPhase) / kWaveSupportPhase;
        motion.step = std::cos(t * M_PI) * wave_params_.gait_step_length;
        motion.lift = 0.0 * units::m;
    }
    return motion;
}

CMoveCombinedGait::LegMotion CMoveCombinedGait::computeRippleLegMotion(ELegIndex index, double phase) const {
    double phase_offset = 0.0;
    for (const auto& info : kRippleLegPhases) {
        if (info.index == index) {
            phase_offset = info.phase_offset;
            break;
        }
    }

    double local_phase = std::fmod(phase + phase_offset, 2.0 * M_PI);
    if (local_phase < 0.0) local_phase += 2.0 * M_PI;

    LegMotion motion;
    if (local_phase < kRippleTransferPhase) {
        double t = local_phase / kRippleTransferPhase;
        motion.step = -std::cos(t * M_PI) * ripple_params_.gait_step_length;
        motion.lift = std::sin(t * M_PI) * ripple_params_.leg_lift_height;
    } else {
        double t = (local_phase - kRippleTransferPhase) / kRippleSupportPhase;
        motion.step = std::cos(t * M_PI) * ripple_params_.gait_step_length;
        motion.lift = 0.0 * units::m;
    }
    return motion;
}

CMoveCombinedGait::LegMotion CMoveCombinedGait::computeTripodLegMotion(ELegIndex index, double phase) const {
    bool is_first = std::ranges::contains(kTripodFirstGroup, index);
    double phase_offset = is_first ? 0.0 : M_PI;
    double phase_with_offset = phase + phase_offset;

    LegMotion motion;
    motion.step = tripod_params_.gait_step_length * std::sin(phase_with_offset);
    motion.lift = tripod_params_.leg_lift_height * std::pow(std::max(0.0, std::cos(phase_with_offset)), 2);
    return motion;
}

CMoveCombinedGait::LegMotion CMoveCombinedGait::computeLegMotion(EMoveCombinedGaitType gait, ELegIndex index,
                                                                 double phase) const {
    switch (gait) {
        case EMoveCombinedGaitType::Wave:
            return computeWaveLegMotion(index, phase);
        case EMoveCombinedGaitType::Ripple:
            return computeRippleLegMotion(index, phase);
        case EMoveCombinedGaitType::Tripod:
            return computeTripodLegMotion(index, phase);
    }
    return {};
}

double CMoveCombinedGait::getVelocityToPhaseGain(EMoveCombinedGaitType gait) const {
    switch (gait) {
        case EMoveCombinedGaitType::Wave:
            return wave_params_.velocity_to_phase_gain;
        case EMoveCombinedGaitType::Ripple:
            return ripple_params_.velocity_to_phase_gain;
        case EMoveCombinedGaitType::Tripod:
            return tripod_params_.velocity_to_phase_gain;
    }
    return wave_params_.velocity_to_phase_gain;
}

units::Angle CMoveCombinedGait::getHeadYawAmplitude(EMoveCombinedGaitType gait) const {
    switch (gait) {
        case EMoveCombinedGaitType::Wave:
            return wave_params_.head_yaw_amplitude;
        case EMoveCombinedGaitType::Ripple:
            return ripple_params_.head_yaw_amplitude;
        case EMoveCombinedGaitType::Tripod:
            return tripod_params_.head_yaw_amplitude;
    }
    return wave_params_.head_yaw_amplitude;
}

bool CMoveCombinedGait::update(const geometry_msgs::msg::Twist& velocity, const CPose& torso,
                               const COrientation& /*head*/) {
    if (state_ == EGaitState::Stopped) {
        return false;
    }

    if (utils::isTwistZero(velocity) && state_ == EGaitState::Running && torso != torso_old_) {
        const auto base_toe_pos = kinematics_->getStandingToePositions();
        kinematics_->moveTorso(base_toe_pos, torso);
        torso_old_ = torso;
        return true;
    }

    if (utils::isTwistZero(velocity) && state_ == EGaitState::Running) {
        if (!no_velocity_timer_.isRunning()) {
            no_velocity_timer_.start();
        } else if (no_velocity_timer_.haveSecondsElapsed(kCombinedTimeToWaitBeforeStopSec)) {
            requestStop();
            no_velocity_timer_.stop();
        }
        return false;
    }

    // Reset idle timer when velocity resumes
    no_velocity_timer_.stop();

    // Filter and store the last non-zero velocity
    if (!utils::isTwistZero(velocity)) {
        velocity_ =
            utils::limitChangeRateUpTwist(velocity_, velocity, combined_params_.velocity_filter_alpha);
    }

    double linear_x = velocity_.linear.x;
    double linear_y = velocity_.linear.y;
    double angular_z = velocity_.angular.z;

    double combined_mag = std::sqrt((linear_x * linear_x) + (linear_y * linear_y) +
                                    (combined_params_.rotation_weight * angular_z * angular_z));

    if (combined_mag < 1e-6) {
        if (state_ != EGaitState::StopPending) return false;
        finishStop(torso);
        return true;
    }

    double norm_x = linear_x / combined_mag;
    double norm_y = linear_y / combined_mag;
    double norm_rot = angular_z / combined_mag;

    // Normalize combined_mag to 0-1 range for gait selection (thresholds are in normalized units)
    double max_combined_mag =
        std::sqrt((combined_params_.max_linear_velocity * combined_params_.max_linear_velocity) * 2.0 +
                  combined_params_.rotation_weight *
                      (combined_params_.max_angular_velocity * combined_params_.max_angular_velocity));
    double normalized_mag =
        (max_combined_mag > 1e-6) ? std::clamp(combined_mag / max_combined_mag, 0.0, 1.0) : 0.0;

    // Select gait based on velocity with hysteresis (only when running, not during stop sequence)
    if (state_ == EGaitState::Running) {
        EMoveCombinedGaitType new_gait = selectGait(normalized_mag);
        if (new_gait != active_gait_type_) {
            RCLCPP_INFO(
                node_->get_logger(),
                "CMoveCombinedGait::update: Switching from %s to %s (normalized: %.3f, velocity: %.4f)",
                magic_enum::enum_name(active_gait_type_).data(), magic_enum::enum_name(new_gait).data(),
                normalized_mag, combined_mag);

            // Capture current positions and head for smooth blending
            blend_start_positions_ = target_positions_;
            blend_start_head_yaw_ = getHeadYawAmplitude(active_gait_type_) * std::sin(phase_);

            active_gait_type_ = new_gait;
            is_blending_ = true;
            blend_alpha_ = 0.0;
        }
    }

    double delta_phase = getVelocityToPhaseGain(active_gait_type_) * combined_mag;
    phase_ += delta_phase;
    phase_ = std::fmod(phase_, 2.0 * M_PI);

    // State transitions
    if (state_ == EGaitState::Starting && phase_ > M_PI / 6.0) {
        state_ = EGaitState::Running;
    }
    if (state_ == EGaitState::StopPending && utils::isSinValueNearZero(phase_, delta_phase)) {
        RCLCPP_INFO(node_->get_logger(), "CMoveCombinedGait::update: Transitioning to Stopped, phase_: %.2f",
                    phase_);
        finishStop(torso);
        return true;
    }

    // Update blend progress
    if (is_blending_) {
        blend_alpha_ += delta_phase / combined_params_.transition_phase_span;
        if (blend_alpha_ >= 1.0) {
            blend_alpha_ = 1.0;
            is_blending_ = false;
        }
    }

    // Cosine interpolation for smooth blending (avoids jerk at start/end)
    double smooth_alpha = is_blending_ ? 0.5 * (1.0 - std::cos(blend_alpha_ * M_PI)) : 1.0;

    const auto standing_positions = kinematics_->getStandingToePositions();

    for (auto& [index, leg] : kinematics_->getLegs()) {
        const auto base_toe_pos = standing_positions.at(index);
        auto motion = computeLegMotion(active_gait_type_, index, phase_);

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

        if (is_blending_) {
            const auto& start_pos = blend_start_positions_.at(index);
            target_positions_[index] = start_pos.linearInterpolate(new_pos, smooth_alpha);
        } else {
            target_positions_[index] = new_pos;
        }
    }

    kinematics_->moveTorso(target_positions_, torso);

    // Head movement with blending
    auto head_yaw = getHeadYawAmplitude(active_gait_type_) * std::sin(phase_);
    if (is_blending_) {
        head_yaw = blend_start_head_yaw_ + smooth_alpha * (head_yaw - blend_start_head_yaw_);
    }
    COrientation head_request;
    head_request.yaw = head_yaw;
    kinematics_->setHeadOrientation(head_request);
    return true;
}

void CMoveCombinedGait::finishStop(const CPose& torso) {
    phase_ = 0.0;
    state_ = EGaitState::Stopped;
    no_velocity_timer_.stop();
    kinematics_->moveTorso(kinematics_->getStandingToePositions(), torso);
    kinematics_->setHeadOrientation(COrientation());
}

void CMoveCombinedGait::requestStop() {
    if (state_ == EGaitState::Starting || state_ == EGaitState::Running) {
        resume_state_ = state_;
        state_ = EGaitState::StopPending;
    }
}

void CMoveCombinedGait::cancelStop() {
    if (state_ == EGaitState::StopPending) {
        state_ = resume_state_;
    }
}

}  // namespace brain
