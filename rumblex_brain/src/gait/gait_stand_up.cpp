#include "gait/gait_stand_up.hpp"

#include <algorithm>
#include <cmath>
#include <map>

#include "gait/pose_model.hpp"

constexpr double kPhaseLimit = M_PI_2;
constexpr double kUpdateIntervalS = 0.1;  // Update interval in seconds

namespace brain {

CStandUpGait::CStandUpGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> kinematics,
                           Parameters::StandUp& params)
    : node_(std::move(node)), kinematics_(std::move(kinematics)), params_(params) {
    target_leg_positions_ = kinematics_->getStandingToePositions();
    target_head_position_ = COrientation(0.0, 0.0, 0.0);
}

void CStandUpGait::start(double duration_s, uint8_t /*direction*/) {
    RCLCPP_INFO(node_->get_logger(), "CStandUpGait::start called, beginning standup gait.");
    phase_ = 0.0;
    phase_increment_ = kPhaseLimit / (duration_s / kUpdateIntervalS);
    origin_leg_positions_ = kinematics_->getToePositions();
    origin_head_orientation_ = kinematics_->getHeadOrientation();

    if (origin_leg_positions_ == target_leg_positions_ && origin_head_orientation_ == target_head_position_) {
        RCLCPP_INFO(
            node_->get_logger(),
            "CStandUpGait::start called, but robot is already in standing position. No action taken.");
        state_ = EGaitState::Stopped;
        return;
    }
    state_ = EGaitState::Running;
}

bool CStandUpGait::update() {
    if (state_ == EGaitState::Stopped) {
        return false;
    }

    phase_ += phase_increment_;
    if (phase_ > kPhaseLimit) {
        phase_ = kPhaseLimit;
        RCLCPP_INFO(node_->get_logger(), "CStandUpGait::update completed, standup finished.");
        state_ = EGaitState::Stopped;
    }

    // progress in [0,1]
    const double progress = std::sin(phase_) * std::sin(phase_);

    // progress 0 means origin position
    // progress 1 means standing position
    std::map<ELegIndex, CPosition> intermediate_positions;
    for (const auto& [leg_index, target_position] : target_leg_positions_) {
        auto origin_position = origin_leg_positions_.at(leg_index);
        // use CPosition member interpolation instead of per-component scalar interpolation
        CPosition intermediate_position = origin_position.linearInterpolate(target_position, progress);
        intermediate_positions[leg_index] = intermediate_position;
    }
    kinematics_->moveTorso(intermediate_positions);

    // Move head to neutral position
    COrientation intermediate_head =
        origin_head_orientation_.linearInterpolate(target_head_position_, progress);
    kinematics_->setHeadOrientation(intermediate_head);

    return true;
}

void CStandUpGait::requestStop() {
    RCLCPP_WARN(node_->get_logger(),
                "CStandUpGait::requestStop called, but Gait StandUp cannot be stopped once started.");
}

void CStandUpGait::cancelStop() {
}

}  // namespace brain
