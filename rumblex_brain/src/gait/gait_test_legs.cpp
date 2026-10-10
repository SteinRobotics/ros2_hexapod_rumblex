#include "gait/gait_test_legs.hpp"

#include <algorithm>
#include <limits>
#include <magic_enum/magic_enum.hpp>

#include "gait/pose_model.hpp"
#include "gait/trajectory.hpp"

namespace brain {
CTestLegsGait::CTestLegsGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> kinematics,
                             Parameters::TestLegs& params)
    : node_(std::move(node)), kinematics_(std::move(kinematics)), params_(params) {
}

void CTestLegsGait::start(units::Duration duration_s, uint8_t /*direction*/) {
    captureBaseAngles();

    current_leg_index_ = 0;
    stage_ = Stage::Raise;
    stage_start_time_ = node_->now();

    const auto requested = duration_s > 0.0 * units::s ? duration_s / 3.0 : default_stage_duration_;
    stage_duration_ = std::max(requested, 1.0 * units::s);

    state_ = hasCurrentLeg() ? EGaitState::Running : EGaitState::Stopped;
}

bool CTestLegsGait::update() {
    if (state_ == EGaitState::Stopped) {
        return false;
    }

    const auto now = node_->get_clock()->now();
    const auto elapsed = std::max(0.0, (now - stage_start_time_).seconds()) * units::s;
    const double progress = trajectoryProgress((elapsed / stage_duration_).numerical_value_in(mp_units::one));
    const auto index = currentLeg();
    auto raised = base_leg_angles_.at(index);
    raised.torso_coxa += params_.torso_coxa_delta;
    raised.coxa_femur += params_.coxa_femur_delta;
    raised.femur_tibia += params_.femur_tibia_delta;
    const double alpha = stage_ == Stage::Raise ? progress : stage_ == Stage::Hold ? 1.0 : 1.0 - progress;
    kinematics_->setLegAngles(index, base_leg_angles_.at(index).linearInterpolate(raised, alpha));
    if (elapsed >= stage_duration_) {
        const bool finished_leg = stage_ == Stage::Lower;
        advanceStage();
        stage_start_time_ = now;
        if (finished_leg && state_ == EGaitState::Stopping) state_ = EGaitState::Stopped;
    }
    return true;
}

void CTestLegsGait::requestStop() {
    if (state_ == EGaitState::Running) state_ = EGaitState::Stopping;
}

void CTestLegsGait::cancelStop() {
    if (state_ == EGaitState::Stopping) {
        state_ = EGaitState::Running;
    }
}

void CTestLegsGait::captureBaseAngles() {
    base_leg_angles_.clear();
    for (const auto& [index, leg] : kinematics_->getLegs()) {
        base_leg_angles_[index] = leg.angles;
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

}  // namespace brain
