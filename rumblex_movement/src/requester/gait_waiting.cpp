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

    const auto base_foot_pos = kinematics_->getLegsStandingPositions();
    auto body_target = CPose();

    constexpr double kBodyLiftHeight = 0.05;                      // 5 cm body lift for visual effect
    body_target.position.z = kBodyLiftHeight * std::sin(phase_);  // Small body bounce for visual effect

    kinematics_->moveBody(base_foot_pos, body_target);
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
