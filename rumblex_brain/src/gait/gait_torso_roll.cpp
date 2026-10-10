#include "gait/gait_torso_roll.hpp"

#include "gait/trajectory.hpp"

constexpr auto kPhaseLimit = TWO_PI * brain::units::rad;

namespace brain {

CTorsoRollGait::CTorsoRollGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> kinematics,
                               Parameters::TorsoRoll& params)
    : node_(node), kinematics_(kinematics), params_(params) {
}

void CTorsoRollGait::start(units::Duration duration_s, uint8_t /*direction*/) {
    duration_s = std::max(duration_s, 1.0 * units::s);
    state_ = EGaitState::Starting;
    phase_ = 0.0 * units::rad;
    phase_rate_ = kPhaseLimit / duration_s;
    torso_origin_ = kinematics_->getTorsoPose();
    origin_leg_positions_ = kinematics_->getToePositions();
}

bool CTorsoRollGait::update() {
    return updateTimed(0.1 * units::s);
}

bool CTorsoRollGait::updateTimed(units::Duration elapsed_s) {
    if (!mp_units::isfinite(elapsed_s) || elapsed_s <= 0.0 * units::s || elapsed_s > 0.5 * units::s)
        return false;
    if (state_ == EGaitState::Stopped) {
        return false;
    }
    phase_ = std::min<units::Angle>(phase_ + phase_rate_ * elapsed_s, kPhaseLimit);
    if (phase_ >= kPhaseLimit - 1e-12 * units::rad) phase_ = kPhaseLimit;
    const auto angle =
        kPhaseLimit * trajectoryProgress((phase_ / kPhaseLimit).numerical_value_in(mp_units::one));
    auto torso = torso_origin_;
    if (phase_ < kPhaseLimit) {
        torso.orientation.roll +=
            params_.torso_max_roll * mp_units::angular::sin(angle).numerical_value_in(mp_units::one);
        torso.orientation.pitch += params_.torso_max_pitch *
                                   (1.0 - mp_units::angular::cos(angle).numerical_value_in(mp_units::one)) *
                                   0.5;
    }
    kinematics_->moveTorso(origin_leg_positions_, torso);
    if (state_ == EGaitState::Starting) state_ = EGaitState::Running;
    if (phase_ >= kPhaseLimit) {
        phase_ = 0.0 * units::rad;
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
