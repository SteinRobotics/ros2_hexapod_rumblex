#include "gait/gait_high_five.hpp"

#include <algorithm>

#include "gait/pose_model.hpp"
#include "rumblex_utils/linear_interpolation.hpp"

namespace {
constexpr double kPhaseIncrement = 0.1;
constexpr int kHoldIterations = 20;

}  // namespace

namespace brain {

CHighFiveGait::CHighFiveGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> kinematics,
                             Parameters::HighFive& params)
    : node_(std::move(node)), kinematics_(std::move(kinematics)), params_(params) {
}

void CHighFiveGait::start(double /*duration_s*/, uint8_t /*direction*/) {
    initial_leg_angles_ = kinematics_->getLegAngles(ELegIndex::RightFront);
    initial_head_ = kinematics_->getHeadOrientation();

    phase_ = EPhase::Raising;
    state_ = EGaitState::Running;
    phase_progress_ = 0.0;
    hold_iterations_remaining_ = kHoldIterations;
}

bool CHighFiveGait::update() {
    if (state_ == EGaitState::Stopped) {
        return false;
    }

    switch (phase_) {
        case EPhase::Raising: {
            phase_progress_ = std::min(phase_progress_ + kPhaseIncrement, 1.0);
            applyInterpolatedPose(phase_progress_);
            if (phase_progress_ >= 1.0 - 1e-6) {
                phase_ = EPhase::Holding;
                phase_progress_ = 1.0;
            }
            break;
        }
        case EPhase::Holding: {
            applyInterpolatedPose(1.0);
            if (hold_iterations_remaining_ > 0) {
                --hold_iterations_remaining_;
            }
            if (hold_iterations_remaining_ <= 0 || state_ == EGaitState::Stopping) {
                transitionToLowering();
            }
            break;
        }
        case EPhase::Lowering: {
            phase_progress_ = std::min(phase_progress_ + kPhaseIncrement, 1.0);
            applyInterpolatedPose(1.0 - phase_progress_);
            if (phase_progress_ >= 1.0 - 1e-6) {
                kinematics_->setLegAngles(ELegIndex::RightFront, initial_leg_angles_);
                kinematics_->setHeadOrientation(initial_head_.yaw, initial_head_.pitch);
                phase_ = EPhase::Finished;
            }
            break;
        }
        case EPhase::Finished:
        case EPhase::Idle: {
            state_ = EGaitState::Stopped;
            return false;
        }
    }

    if (phase_ == EPhase::Finished) {
        state_ = EGaitState::Stopped;
        RCLCPP_INFO(node_->get_logger(), "CHighFiveGait::update completed, high five finished.");
        return false;
    }

    if (state_ == EGaitState::Stopping && phase_ != EPhase::Lowering) {
        transitionToLowering();
    } else if (phase_ != EPhase::Finished) {
        state_ = EGaitState::Running;
    }

    return true;
}

void CHighFiveGait::requestStop() {
    if (state_ == EGaitState::Running) {
        state_ = EGaitState::Stopping;
        transitionToLowering();
    }
}

void CHighFiveGait::cancelStop() {
    if (state_ == EGaitState::Stopping) {
        state_ = EGaitState::Running;
    }
}

void CHighFiveGait::applyInterpolatedPose(double alpha) {
    alpha = std::clamp(alpha, 0.0, 1.0);
    // use CLegAngles member interpolation
    const auto leg_angles = initial_leg_angles_.linearInterpolate(target_leg_angles_, alpha);
    kinematics_->setLegAngles(ELegIndex::RightFront, leg_angles);

    const auto head_yaw = initial_head_.yaw + (target_head_yaw_ - initial_head_.yaw) * alpha;
    const auto head_pitch = initial_head_.pitch + (target_head_pitch_ - initial_head_.pitch) * alpha;
    kinematics_->setHeadOrientation(head_yaw, head_pitch);
}

void CHighFiveGait::transitionToLowering() {
    phase_ = EPhase::Lowering;
    phase_progress_ = 0.0;
}

}  // namespace brain
