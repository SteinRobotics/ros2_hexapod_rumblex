#include "gait/gait_torso_roll.hpp"

#include "gait/trajectory.hpp"

constexpr double kPhaseLimit = TWO_PI;

namespace brain {

CTorsoRollGait::CTorsoRollGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> kinematics,
                               Parameters::TorsoRoll& params)
    : node_(node), kinematics_(kinematics), params_(params) {
}

void CTorsoRollGait::start(double duration_s, uint8_t /*direction*/) {
    duration_s = std::max(duration_s, 1.0);
    state_ = EGaitState::Starting;
    phase_ = 0.0;
    phase_rate_ = kPhaseLimit / duration_s;
    torso_origin_ = kinematics_->getTorsoPose();
    origin_leg_positions_ = kinematics_->getToePositions();
}

bool CTorsoRollGait::update() {
    return updateTimed(0.1);
}

bool CTorsoRollGait::updateTimed(double elapsed_s) {
    if (!std::isfinite(elapsed_s) || elapsed_s <= 0.0 || elapsed_s > 0.5) return false;
    if (state_ == EGaitState::Stopped) {
        return false;
    }
    phase_ = std::min(phase_ + phase_rate_ * elapsed_s, kPhaseLimit);
    if (phase_ >= kPhaseLimit - 1e-12) phase_ = kPhaseLimit;
    const double angle = kPhaseLimit * trajectoryProgress(phase_ / kPhaseLimit);
    auto torso = torso_origin_;
    if (phase_ < kPhaseLimit) {
        torso.orientation.roll += params_.torso_max_roll * std::sin(angle);
        torso.orientation.pitch += params_.torso_max_pitch * (1.0 - std::cos(angle)) * 0.5;
    }
    kinematics_->moveTorso(origin_leg_positions_, torso);
    if (state_ == EGaitState::Starting) state_ = EGaitState::Running;
    if (phase_ >= kPhaseLimit) {
        phase_ = 0.0;
        if (state_ == EGaitState::StopPending) state_ = EGaitState::Stopped;
    }
    return true;
}

void CTorsoRollGait::requestStop() {
    if (state_ == EGaitState::Starting || state_ == EGaitState::Running) {
        resume_state_ = state_;
        state_ = EGaitState::StopPending;
    }
}

void CTorsoRollGait::cancelStop() {
    // if the state is not in state StopPending, cancel the transition to Stop is not possible
    if (state_ == EGaitState::StopPending) {
        state_ = resume_state_;
    }
}

}  // namespace brain
