#include "requester/gait_torso_roll.hpp"

constexpr double kPhaseLimit = TWO_PI;
constexpr double kUpdateIntervalS = 0.1;  // Update interval in seconds

namespace rumblex_movement {

CTorsoRollGait::CTorsoRollGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                               Parameters::TorsoRoll& params)
    : node_(node), kinematics_(kinematics), params_(params) {
}

void CTorsoRollGait::start(double duration_s, uint8_t /*direction*/) {
    assert(duration_s > 0.0 && "CTorsoRollGait::start duration must be positive.");
    state_ = EGaitState::Starting;
    phase_ = 0.0;
    phase_increment_ = kPhaseLimit / (duration_s / kUpdateIntervalS);
    origin_leg_positions_ = kinematics_->getToePositions();
}

bool CTorsoRollGait::update() {
    if (state_ == EGaitState::Stopped) {
        return false;
    }
    phase_ += phase_increment_;
    phase_ = std::fmod(phase_, kPhaseLimit);

    // phase_ == M_PI_4 is reached when the leg is moving upwards and the normal cycle goes downwards again
    if (state_ == EGaitState::Starting && phase_ > M_PI_4) {
        RCLCPP_INFO(node_->get_logger(), "CTorsoRollGait change to Running.");
        state_ = EGaitState::Running;
    }
    if (state_ == EGaitState::StopPending && utils::areSinCosValuesEqual(phase_, phase_increment_)) {
        RCLCPP_INFO(node_->get_logger(), "CTorsoRollGait change to Stopping.");
        state_ = EGaitState::Stopping;
    } else if (state_ == EGaitState::Stopping && utils::isSinValueNearZero(phase_, phase_increment_)) {
        RCLCPP_INFO(node_->get_logger(), "CTorsoRollGait change to Stopped.");
        phase_ = 0.0;
        state_ = EGaitState::Stopped;
    }

    auto torso = CPose();

    // Roll is always a sine wave
    torso.orientation.roll = params_.torso_max_roll * std::sin(phase_);

    // Pitch behavior depends on state
    if (state_ == EGaitState::Running || state_ == EGaitState::StopPending) {
        torso.orientation.pitch = params_.torso_max_pitch * std::cos(phase_);
    } else {
        // phase_ == M_PI_4 is reached when the leg is moving upwards and the normal cycle goes downwards again
        torso.orientation.pitch = params_.torso_max_pitch * std::sin(phase_);
    }

    kinematics_->moveTorso(origin_leg_positions_, torso);
    return true;
}

void CTorsoRollGait::requestStop() {
    if (state_ == EGaitState::Running) {
        state_ = EGaitState::StopPending;
    }
}

void CTorsoRollGait::cancelStop() {
    // if the state is not in state StopPending, cancel the transition to Stop is not possible
    if (state_ == EGaitState::StopPending) {
        state_ = EGaitState::Running;
    }
}

}  // namespace rumblex_movement
