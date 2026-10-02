#include "gait/gait_single_pose.hpp"

using namespace rumblex_interfaces::msg;
namespace brain {

CSinglePoseGait::CSinglePoseGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> kinematics,
                                 Parameters::SinglePose& params)
    : node_(node), kinematics_(kinematics), params_(params) {
}

void CSinglePoseGait::start(double duration_s, uint8_t /*direction*/) {
    RCLCPP_INFO(node_->get_logger(), "Starting CSinglePoseGait");
    state_ = EGaitState::Running;
    phase_ = 0.0;
    duration_s_ = duration_s;

    torso_origin_ = kinematics_->getTorsoPose();
    head_origin_ = kinematics_->getHeadOrientation();

    // 100ms task update time, duration in seconds,
    phase_increment_ = duration_s_ * 0.1;
}

bool CSinglePoseGait::update(const geometry_msgs::msg::Twist& /*velocity*/, const CPose& torso,
                             const COrientation& head) {
    if (state_ == EGaitState::Stopped) return false;

    // set head yaw using sinusoidal oscillation
    phase_ += phase_increment_;
    if (phase_ > duration_s_) {
        state_ = EGaitState::Stopped;
        kinematics_->setHeadOrientation(head);
        kinematics_->moveTorso(torso);
        return true;
    }
    // interpolate torso position from origin to target
    auto progress = phase_ / duration_s_;
    CPose intermediate_pose = torso_origin_.linearInterpolate(torso, progress);
    COrientation intermediate_head = head_origin_.linearInterpolate(head, progress);

    kinematics_->moveTorso(intermediate_pose);
    kinematics_->setHeadOrientation(intermediate_head);

    return true;
}

void CSinglePoseGait::requestStop() {
    // gait is stopped automatically after completing the current cycle
}

void CSinglePoseGait::cancelStop() {
    // this gait cannot be stopped nor the stop can be cancelled
}

}  // namespace brain
