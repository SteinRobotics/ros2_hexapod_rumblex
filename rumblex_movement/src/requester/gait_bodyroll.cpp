#include "requester/gait_bodyroll.hpp"

constexpr double kPhaseLimit = TWO_PI;
constexpr double kUpdateIntervalS = 0.1;  // Update interval in seconds

namespace rumblex_movement {

CGaitBodyRoll::CGaitBodyRoll(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                             Parameters::BodyRoll& params)
    : node_(node), kinematics_(kinematics), params_(params) {
}

void CGaitBodyRoll::start(double duration_s, uint8_t /*direction*/) {
    assert(duration_s > 0.0 && "CGaitBodyRoll::start duration must be positive.");
    state_ = EGaitState::Starting;
    phase_ = 0.0;
    phase_increment_ = kPhaseLimit / (duration_s / kUpdateIntervalS);
    origin_leg_positions_ = kinematics_->getLegsPositions();
}

bool CGaitBodyRoll::update() {
    if (state_ == EGaitState::Stopped) {
        return false;
    }
    phase_ += phase_increment_;
    phase_ = std::fmod(phase_, kPhaseLimit);

    // phase_ == M_PI_4 is reached when the leg is moving upwards and the normal cycle goes downwards again
    if (state_ == EGaitState::Starting && phase_ > M_PI_4) {
        RCLCPP_INFO(node_->get_logger(), "CGaitBodyRoll change to Running.");
        state_ = EGaitState::Running;
    }
    if (state_ == EGaitState::StopPending && utils::areSinCosValuesEqual(phase_, phase_increment_)) {
        RCLCPP_INFO(node_->get_logger(), "CGaitBodyRoll change to Stopping.");
        state_ = EGaitState::Stopping;
    } else if (state_ == EGaitState::Stopping && utils::isSinValueNearZero(phase_, phase_increment_)) {
        RCLCPP_INFO(node_->get_logger(), "CGaitBodyRoll change to Stopped.");
        phase_ = 0.0;
        state_ = EGaitState::Stopped;
    }

    auto body = CPose();

    // Roll is always a sine wave
    body.orientation.roll_deg = params_.body_max_roll_deg * std::sin(phase_);

    // Pitch behavior depends on state
    if (state_ == EGaitState::Running || state_ == EGaitState::StopPending) {
        body.orientation.pitch_deg = params_.body_max_pitch_deg * std::cos(phase_);
    } else {
        // phase_ == M_PI_4 is reached when the leg is moving upwards and the normal cycle goes downwards again
        body.orientation.pitch_deg = params_.body_max_pitch_deg * std::sin(phase_);
    }

    kinematics_->moveBody(origin_leg_positions_, body);
    return true;
}

void CGaitBodyRoll::requestStop() {
    if (state_ == EGaitState::Running) {
        state_ = EGaitState::StopPending;
    }
}

void CGaitBodyRoll::cancelStop() {
    // if the state is not in state StopPending, cancel the transition to Stop is not possible
    if (state_ == EGaitState::StopPending) {
        state_ = EGaitState::Running;
    }
}

}  // namespace rumblex_movement
