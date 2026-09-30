#include "requester/gait_waiting.hpp"

#include "requester/kinematics.hpp"

namespace rumblex_movement {
CWaitingGait::CWaitingGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                           Parameters::Waiting& params)
    : node_(node), kinematics_(kinematics), params_(params) {
}

void CWaitingGait::start(double /*duration_s*/, uint8_t /*direction*/) {
    // Directly enter Running; no distinct Starting phase needed.
    state_ = EGaitState::Running;
    phase_ = 0.0;
}

bool CWaitingGait::update() {
    if (state_ == EGaitState::Stopped) {
        return false;
    }
    constexpr double kDeltaPhase = 0.1;
    phase_ += kDeltaPhase;

    // When stopping, finish current lift cycle cleanly.
    if (state_ == EGaitState::Stopping && utils::isSinValueNearZero(phase_, kDeltaPhase)) {
        state_ = EGaitState::Stopped;
        return false;
    }

    const auto base_toe_pos = kinematics_->getStandingToePositions();
    auto torso_target = CPose();

    constexpr auto kTorsoLiftHeight = 0.05 * rumblex_movement::units::m;  // 5 cm torso lift for visual effect
    torso_target.position.z = kTorsoLiftHeight * std::sin(phase_);  // Small torso bounce for visual effect

    kinematics_->moveTorso(base_toe_pos, torso_target);
    return true;
}

void CWaitingGait::requestStop() {
    if (state_ == EGaitState::Running) {
        state_ = EGaitState::Stopping;
    }
}

void CWaitingGait::cancelStop() {
    if (state_ == EGaitState::Stopping) {
        state_ = EGaitState::Running;
    }
}

}  // namespace rumblex_movement
