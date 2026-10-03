#include "gait/gait_single_pose.hpp"

#include <algorithm>
#include <cmath>

using namespace rumblex_interfaces::msg;
namespace brain {

CSinglePoseGait::CSinglePoseGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> kinematics,
                                 Parameters::SinglePose& /*params*/, std::optional<Target> target)
    : node_(node), kinematics_(kinematics), target_(std::move(target)) {
}

void CSinglePoseGait::start(double duration_s, uint8_t /*direction*/) {
    RCLCPP_INFO(node_->get_logger(), "Starting CSinglePoseGait");
    state_ = EGaitState::Running;
    phase_ = 0.0;
    duration_s_ = duration_s;

    torso_origin_ = kinematics_->getTorsoPose();
    head_origin_ = kinematics_->getHeadOrientation();

    toe_origins_ = kinematics_->getToePositions();
    if (target_ && toe_origins_ == target_->toes && torso_origin_ == target_->torso &&
        head_origin_ == target_->head) {
        state_ = EGaitState::Stopped;
    }
}

bool CSinglePoseGait::update(const geometry_msgs::msg::Twist& /*velocity*/, const CPose& torso,
                             const COrientation& head) {
    if (state_ == EGaitState::Stopped) return false;

    // The movement handler updates gaits every 100 ms.
    phase_ += 0.1;
    const double fraction = duration_s_ > 0.0 ? std::min(phase_ / duration_s_, 1.0) : 1.0;
    if (fraction >= 1.0 - 1e-12) state_ = EGaitState::Stopped;
    double progress = state_ == EGaitState::Stopped ? 1.0 : fraction;
    if (target_) {
        // Preserve the smooth easing of fixed pose transitions.
        const double sine = std::sin(progress * M_PI_2);
        progress = sine * sine;
    }
    const auto& target_torso = target_ ? target_->torso : torso;
    const auto& target_head = target_ ? target_->head : head;
    const CPose intermediate_pose = state_ == EGaitState::Stopped
                                        ? target_torso
                                        : torso_origin_.linearInterpolate(target_torso, progress);
    if (target_) {
        std::map<ELegIndex, CPosition> toes;
        for (const auto& [index, position] : target_->toes) {
            toes[index] = state_ == EGaitState::Stopped
                              ? position
                              : toe_origins_.at(index).linearInterpolate(position, progress);
        }
        kinematics_->moveTorso(toes, intermediate_pose);
    } else {
        kinematics_->moveTorso(intermediate_pose);
    }
    kinematics_->setHeadOrientation(
        state_ == EGaitState::Stopped ? target_head : head_origin_.linearInterpolate(target_head, progress));

    return true;
}

void CSinglePoseGait::requestStop() {
    // gait is stopped automatically after completing the current cycle
}

void CSinglePoseGait::cancelStop() {
    // this gait cannot be stopped nor the stop can be cancelled
}

}  // namespace brain
