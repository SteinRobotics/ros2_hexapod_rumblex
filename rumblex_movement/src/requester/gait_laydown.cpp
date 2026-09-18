#include "requester/gait_laydown.hpp"

#include <algorithm>
#include <cmath>
#include <map>

#include "requester/kinematics.hpp"

constexpr double kPhaseLimit = M_PI_2;
constexpr double kUpdateIntervalS = 0.1;  // Update interval in seconds

namespace rumblex_movement {

CLayDownGait::CLayDownGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                           Parameters::LayDown& params)
    : node_(node), kinematics_(kinematics), params_(params) {
    target_leg_positions_ = kinematics_->getLegsLayDownPositions();
    target_head_position_ = COrientation(0.0, -params_.head_max_pitch_deg, 0.0);
}

void CLayDownGait::start(double duration_s, uint8_t /*direction*/) {
    RCLCPP_INFO(node_->get_logger(),
                "CLayDownGait::start called, beginning laydown gait with duration %.2f seconds.", duration_s);
    phase_ = 0.0;
    phase_increment_ = kPhaseLimit / (duration_s / kUpdateIntervalS);
    origin_leg_positions_ = kinematics_->getLegsPositions();
    origin_head_position_ = kinematics_->getHead();

    if (origin_leg_positions_ == target_leg_positions_ && origin_head_position_ == target_head_position_) {
        RCLCPP_INFO(node_->get_logger(),
                    "CLayDownGait::start called, but robot is already in laydown position. No action taken.");
        state_ = EGaitState::Stopped;
        return;
    }
    state_ = EGaitState::Running;
}

bool CLayDownGait::update() {
    if (state_ == EGaitState::Stopped) {
        return false;
    }

    phase_ += phase_increment_;
    if (phase_ > kPhaseLimit) {
        phase_ = kPhaseLimit;
        RCLCPP_INFO(node_->get_logger(), "CLayDownGait::update completed, laydown finished.");
        state_ = EGaitState::Stopped;
    }

    // progress in [0,1]
    const double progress = std::sin(phase_) * std::sin(phase_);

    // progress 0 means origin position
    // progress 1 means laydown position
    std::map<ELegIndex, CPosition> intermediate_positions;
    for (const auto& [legIndex, laydown_pos] : target_leg_positions_) {
        auto origin_pos = origin_leg_positions_.at(legIndex);
        CPosition intermediate_position = origin_pos.linearInterpolate(laydown_pos, progress);
        intermediate_positions[legIndex] = intermediate_position;
    }
    kinematics_->moveBody(intermediate_positions);

    // Move head up
    COrientation intermediate_head = origin_head_position_.linearInterpolate(target_head_position_, progress);
    kinematics_->setHead(intermediate_head);

    return true;
}

void CLayDownGait::requestStop() {
    RCLCPP_WARN(node_->get_logger(),
                "CLayDownGait::requestStop called, but Gait LayDown cannot be stopped once started.");
}

void CLayDownGait::cancelStop() {
}

}  // namespace rumblex_movement
