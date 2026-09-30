#include "requester/gait_highfive.hpp"

#include <algorithm>

#include "requester/kinematics.hpp"
#include "rumblex_utils/linear_interpolation.hpp"

namespace {
constexpr double kPhaseIncrement = 0.1;
constexpr int kHoldIterations = 20;

}  // namespace

namespace rumblex_movement {

CHighFiveGait::CHighFiveGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                             Parameters::HighFive& params)
    : node_(std::move(node)), kinematics_(std::move(kinematics)), params_(params) {
}

void CHighFiveGait::start(double /*duration_s*/, uint8_t /*direction*/) {
    initial_leg_angles_ = kinematics_->getAngles(ELegIndex::RightFront);
    initial_head_ = kinematics_->getHead();

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
                kinematics_->setHead(initial_head_.yaw, initial_head_.pitch);
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
    const auto legAngles = initial_leg_angles_.linearInterpolate(target_leg_angles_, alpha);
    kinematics_->setLegAngles(ELegIndex::RightFront, legAngles);

    const auto headYaw = initial_head_.yaw + (target_head_yaw_ - initial_head_.yaw) * alpha;
    const auto headPitch = initial_head_.pitch + (target_head_pitch_ - initial_head_.pitch) * alpha;
    kinematics_->setHead(headYaw, headPitch);
}

void CHighFiveGait::transitionToLowering() {
    phase_ = EPhase::Lowering;
    phase_progress_ = 0.0;
}

}  // namespace rumblex_movement
