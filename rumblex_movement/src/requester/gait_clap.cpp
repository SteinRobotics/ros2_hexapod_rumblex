#include "requester/gait_clap.hpp"

#include <algorithm>

#include "rumblex_utils/linear_interpolation.hpp"
#include "requester/kinematics.hpp"

namespace {
constexpr double kPhaseIncrement = 0.1;
constexpr double kBodyShiftBack = -0.05;     // 5cm backward
constexpr double kBackLegLiftHeight = 0.05;  // 5cm up
constexpr double kFrontLegLiftHeight = 0.1;  // 10cm up
constexpr double kClapAngle = 30.0;          // Degrees for clap movement
constexpr int kClapRepetitions = 3;          // Number of clap cycles

}  // namespace

namespace rumblex_movement {

CClapGait::CClapGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                     Parameters::Clap& params)
    : node_(node), kinematics_(kinematics), params_(params) {
}

void CClapGait::start(double /*duration_s*/, uint8_t /*direction*/) {
    // Store initial positions
    initial_foot_positions_ = kinematics_->getLegsPositions();
    initial_body_pose_ = kinematics_->getBody();

    phase_ = EPhase::ShiftingBack;
    state_ = EGaitState::Running;
    phase_progress_ = 0.0;
    clap_iterations_remaining_ = kClapRepetitions * 2;  // Open and close for each repetition

    RCLCPP_INFO(node_->get_logger(), "CClapGait::start - Starting clap sequence");
}

bool CClapGait::update() {
    if (state_ == EGaitState::Stopped) {
        return false;
    }

    switch (phase_) {
        case EPhase::ShiftingBack: {
            phase_progress_ = std::min(phase_progress_ + kPhaseIncrement, 1.0);
            applyBodyShift(phase_progress_);
            if (phase_progress_ >= 1.0 - 1e-6) {
                phase_ = EPhase::LiftRightBack;
                phase_progress_ = 0.0;
            }
            break;
        }
        case EPhase::LiftRightBack: {
            phase_progress_ = std::min(phase_progress_ + kPhaseIncrement, 1.0);
            applyBackLegLift(ELegIndex::RightBack, phase_progress_);
            if (phase_progress_ >= 1.0 - 1e-6) {
                phase_ = EPhase::LowerRightBack;
                phase_progress_ = 0.0;
            }
            break;
        }
        case EPhase::LowerRightBack: {
            phase_progress_ = std::min(phase_progress_ + kPhaseIncrement, 1.0);
            applyBackLegLift(ELegIndex::RightBack, 1.0 - phase_progress_);
            if (phase_progress_ >= 1.0 - 1e-6) {
                phase_ = EPhase::LiftLeftBack;
                phase_progress_ = 0.0;
            }
            break;
        }
        case EPhase::LiftLeftBack: {
            phase_progress_ = std::min(phase_progress_ + kPhaseIncrement, 1.0);
            applyBackLegLift(ELegIndex::LeftBack, phase_progress_);
            if (phase_progress_ >= 1.0 - 1e-6) {
                phase_ = EPhase::LowerLeftBack;
                phase_progress_ = 0.0;
            }
            break;
        }
        case EPhase::LowerLeftBack: {
            phase_progress_ = std::min(phase_progress_ + kPhaseIncrement, 1.0);
            applyBackLegLift(ELegIndex::LeftBack, 1.0 - phase_progress_);
            if (phase_progress_ >= 1.0 - 1e-6) {
                phase_ = EPhase::LiftFrontLegs;
                phase_progress_ = 0.0;
            }
            break;
        }
        case EPhase::LiftFrontLegs: {
            phase_progress_ = std::min(phase_progress_ + kPhaseIncrement, 1.0);
            applyFrontLegsLift(phase_progress_);
            if (phase_progress_ >= 1.0 - 1e-6) {
                phase_ = EPhase::ClapClosing;
                phase_progress_ = 0.0;
            }
            break;
        }
        case EPhase::ClapClosing: {
            phase_progress_ = std::min(phase_progress_ + kPhaseIncrement * 2.0, 1.0);  // Faster clap
            applyFrontLegsClap(phase_progress_, true);
            if (phase_progress_ >= 1.0 - 1e-6) {
                phase_ = EPhase::ClapOpening;
                phase_progress_ = 0.0;
                clap_iterations_remaining_--;
            }
            break;
        }
        case EPhase::ClapOpening: {
            phase_progress_ = std::min(phase_progress_ + kPhaseIncrement * 2.0, 1.0);  // Faster clap
            applyFrontLegsClap(1.0 - phase_progress_, true);
            if (phase_progress_ >= 1.0 - 1e-6) {
                if (clap_iterations_remaining_ > 0) {
                    phase_ = EPhase::ClapClosing;
                    phase_progress_ = 0.0;
                } else {
                    phase_ = EPhase::LowerFrontLegs;
                    phase_progress_ = 0.0;
                }
            }
            break;
        }
        case EPhase::LowerFrontLegs: {
            phase_progress_ = std::min(phase_progress_ + kPhaseIncrement, 1.0);
            applyFrontLegsLift(1.0 - phase_progress_);
            if (phase_progress_ >= 1.0 - 1e-6) {
                phase_ = EPhase::ShiftingForward;
                phase_progress_ = 0.0;
            }
            break;
        }
        case EPhase::ShiftingForward: {
            phase_progress_ = std::min(phase_progress_ + kPhaseIncrement, 1.0);
            applyBodyShift(1.0 - phase_progress_);
            if (phase_progress_ >= 1.0 - 1e-6) {
                phase_ = EPhase::Finished;
            }
            break;
        }
        case EPhase::Finished:
        case EPhase::Idle: {
            state_ = EGaitState::Stopped;
            RCLCPP_INFO(node_->get_logger(), "CClapGait::update completed, clap finished.");
            return false;
        }
    }

    if (state_ == EGaitState::Stopping && phase_ != EPhase::ShiftingForward && phase_ != EPhase::Finished) {
        // Transition to return to initial position
        if (phase_ >= EPhase::LiftFrontLegs && phase_ <= EPhase::ClapOpening) {
            // If front legs are up or in clap, lower them first
            phase_ = EPhase::LowerFrontLegs;
            phase_progress_ = 0.0;
        } else {
            // Go straight to shifting forward
            phase_ = EPhase::ShiftingForward;
            phase_progress_ = 0.0;
        }
    }

    return true;
}

void CClapGait::requestStop() {
    if (state_ == EGaitState::Running) {
        state_ = EGaitState::Stopping;
    }
}

void CClapGait::cancelStop() {
    if (state_ == EGaitState::Stopping) {
        state_ = EGaitState::Running;
    }
}

void CClapGait::applyBodyShift(double alpha) {
    alpha = std::clamp(alpha, 0.0, 1.0);

    // Shift body backward
    CPose shifted_body = initial_body_pose_;
    shifted_body.position.x += kBodyShiftBack * alpha;

    // Apply body shift while keeping legs in standing positions
    kinematics_->moveBody(initial_foot_positions_, shifted_body);
}

void CClapGait::applyBackLegLift(ELegIndex leg, double alpha) {
    alpha = std::clamp(alpha, 0.0, 1.0);

    // Lift the specified back leg
    auto current_positions = kinematics_->getLegsPositions();
    CPosition lifted_position = initial_foot_positions_.at(leg);
    lifted_position.z += kBackLegLiftHeight * alpha;

    current_positions[leg] = lifted_position;

    // Apply the lifted position
    kinematics_->setSingleFeet(leg, lifted_position);
}

void CClapGait::applyFrontLegsLift(double alpha) {
    alpha = std::clamp(alpha, 0.0, 1.0);

    // Lift both front legs
    auto left_front_pos = initial_foot_positions_.at(ELegIndex::LeftFront);
    auto right_front_pos = initial_foot_positions_.at(ELegIndex::RightFront);

    left_front_pos.z += kFrontLegLiftHeight * alpha;
    right_front_pos.z += kFrontLegLiftHeight * alpha;

    kinematics_->setSingleFeet(ELegIndex::LeftFront, left_front_pos);
    kinematics_->setSingleFeet(ELegIndex::RightFront, right_front_pos);
}

void CClapGait::applyFrontLegsClap(double alpha, [[maybe_unused]] bool closing) {
    alpha = std::clamp(alpha, 0.0, 1.0);

    // Get current leg angles
    auto left_front_angles = kinematics_->getAngles(ELegIndex::LeftFront);
    auto right_front_angles = kinematics_->getAngles(ELegIndex::RightFront);

    // Adjust coxa angles to bring legs together (closing) or apart (opening)
    // Left leg rotates clockwise (positive), right leg rotates counter-clockwise (negative)
    double angle_offset = kClapAngle * alpha;

    left_front_angles.coxa_deg += angle_offset;
    right_front_angles.coxa_deg -= angle_offset;

    kinematics_->setLegAngles(ELegIndex::LeftFront, left_front_angles);
    kinematics_->setLegAngles(ELegIndex::RightFront, right_front_angles);
}

}  // namespace rumblex_movement
