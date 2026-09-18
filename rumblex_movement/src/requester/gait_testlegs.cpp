#include "requester/gait_testlegs.hpp"

#include <algorithm>
#include <limits>
#include <magic_enum.hpp>

#include "requester/kinematics.hpp"

namespace rumblex_movement {
CTestLegsGait::CTestLegsGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                             Parameters::TestLegs& params)
    : node_(std::move(node)), kinematics_(std::move(kinematics)), params_(params) {
}

void CTestLegsGait::start(double duration_s, uint8_t /*direction*/) {
    captureBaseAngles();

    current_leg_index_ = 0;
    stage_ = Stage::Raise;
    stage_action_applied_ = false;
    stage_start_time_ = node_->now();

    const double requested = duration_s > 0.0 ? duration_s / 3.0 : default_stage_duration_;
    stage_duration_ = std::max(requested, min_stage_duration_);

    state_ = hasCurrentLeg() ? EGaitState::Running : EGaitState::Stopped;
}

bool CTestLegsGait::update() {
    if (state_ == EGaitState::Stopped) {
        return false;
    }

    const auto now = node_->get_clock()->now();

    if (!stage_action_applied_) {
        applyStageForCurrentLeg();
        stage_action_applied_ = true;
        stage_start_time_ = now;
    } else {
        const auto elapsed = (now - stage_start_time_).seconds();
        if (elapsed >= stage_duration_) {
            advanceStage();
            stage_start_time_ = now;
            stage_action_applied_ = false;
            if (state_ == EGaitState::Stopped) {
                // The last stage finished in this update call. Signal success so callers know
                // the gait progressed, even though it is now stopped.
                return true;
            }
            applyStageForCurrentLeg();
            stage_action_applied_ = true;
        }
    }

    return true;
}

void CTestLegsGait::requestStop() {
    if (state_ == EGaitState::Stopped) {
        return;
    }
    restoreAllLegs();
    state_ = EGaitState::Stopped;
}

void CTestLegsGait::cancelStop() {
    if (state_ == EGaitState::Stopping) {
        state_ = EGaitState::Running;
        stage_start_time_ = node_->now();
        stage_action_applied_ = false;
    }
}

void CTestLegsGait::captureBaseAngles() {
    base_leg_angles_.clear();
    for (const auto& [index, leg] : kinematics_->getLegs()) {
        base_leg_angles_[index] = leg.angles_deg_;
    }

    leg_order_.assign(kDefaultLegOrder.begin(), kDefaultLegOrder.end());
    // Ensure we only iterate over legs that exist in the kinematics map
    std::erase_if(leg_order_, [this](ELegIndex idx) { return base_leg_angles_.count(idx) == 0; });
    if (leg_order_.empty()) {
        for (const auto& [index, _] : base_leg_angles_) {
            leg_order_.push_back(index);
        }
    }
}

void CTestLegsGait::applyStageForCurrentLeg() {
    if (!hasCurrentLeg()) {
        state_ = EGaitState::Stopped;
        return;
    }

    const auto index = currentLeg();
    const auto it = base_leg_angles_.find(index);
    if (it == base_leg_angles_.end()) {
        state_ = EGaitState::Stopped;
        return;
    }

    auto target = it->second;
    switch (stage_) {
        case Stage::Raise:
            target.coxa_deg += params_.coxa_delta_deg;
            target.femur_deg += params_.femur_delta_deg;
            target.tibia_deg += params_.tibia_delta_deg;
            RCLCPP_INFO_STREAM(node_->get_logger(), "CTestLegsGait: raising "
                                                        << magic_enum::enum_name(index) << " by ("
                                                        << params_.coxa_delta_deg << ", "
                                                        << params_.femur_delta_deg << ", "
                                                        << params_.tibia_delta_deg << ") degrees");
            kinematics_->setLegAngles(index, target);
            break;
        case Stage::Hold:
            RCLCPP_DEBUG_STREAM(node_->get_logger(),
                                "CTestLegsGait: holding " << magic_enum::enum_name(index));
            break;
        case Stage::Lower:
            RCLCPP_INFO_STREAM(node_->get_logger(),
                               "CTestLegsGait: lowering " << magic_enum::enum_name(index));
            kinematics_->setLegAngles(index, it->second);
            break;
    }
}

void CTestLegsGait::advanceStage() {
    if (!hasCurrentLeg()) {
        state_ = EGaitState::Stopped;
        return;
    }

    switch (stage_) {
        case Stage::Raise:
            stage_ = Stage::Hold;
            break;
        case Stage::Hold:
            stage_ = Stage::Lower;
            break;
        case Stage::Lower:
            ++current_leg_index_;
            stage_ = Stage::Raise;
            if (!hasCurrentLeg()) {
                state_ = EGaitState::Stopped;
            }
            break;
    }
}

bool CTestLegsGait::hasCurrentLeg() const {
    return current_leg_index_ < leg_order_.size();
}

ELegIndex CTestLegsGait::currentLeg() const {
    return leg_order_.at(current_leg_index_);
}

void CTestLegsGait::restoreLeg(ELegIndex index) {
    const auto it = base_leg_angles_.find(index);
    if (it != base_leg_angles_.end()) {
        kinematics_->setLegAngles(index, it->second);
    }
}

void CTestLegsGait::restoreAllLegs() {
    for (const auto& [index, angles] : base_leg_angles_) {
        kinematics_->setLegAngles(index, angles);
    }
    stage_action_applied_ = false;
    current_leg_index_ = 0;
    stage_ = Stage::Raise;
}

}  // namespace rumblex_movement
