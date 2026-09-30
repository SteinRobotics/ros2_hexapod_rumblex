#include "requester/gait_singlepose.hpp"

using namespace rumblex_interfaces::msg;
namespace rumblex_movement {

CGaitSinglePose::CGaitSinglePose(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                                 Parameters::SinglePose& params)
    : node_(node), kinematics_(kinematics), params_(params) {
}

void CGaitSinglePose::start(double duration_s, uint8_t /*direction*/) {
    RCLCPP_INFO(node_->get_logger(), "Starting CGaitSinglePose");
    state_ = EGaitState::Running;
    phase_ = 0.0;
    duration_s_ = duration_s;

    torso_origin_ = kinematics_->getTorso();
    head_origin_ = kinematics_->getHead();

    // 100ms task update time, duration in seconds,
    phase_increment_ = duration_s_ * 0.1;
}

bool CGaitSinglePose::update(const geometry_msgs::msg::Twist& /*velocity*/, const CPose& torso,
                             const COrientation& head) {
    if (state_ == EGaitState::Stopped) return false;

    // set head yaw using sinusoidal oscillation
    phase_ += phase_increment_;
    if (phase_ > duration_s_) {
        state_ = EGaitState::Stopped;
        kinematics_->setHead(head);
        kinematics_->moveTorso(torso);
        return true;
    }
    // interpolate torso position from origin to target
    auto progress = phase_ / duration_s_;
    CPose intermediate_pose = torso_origin_.linearInterpolate(torso, progress);
    COrientation intermediate_head = head_origin_.linearInterpolate(head, progress);

    kinematics_->moveTorso(intermediate_pose);
    kinematics_->setHead(intermediate_head);

    return true;
}

void CGaitSinglePose::requestStop() {
    // gait is stopped automatically after completing the current cycle
}

void CGaitSinglePose::cancelStop() {
    // this gait cannot be stopped nor the stop can be cancelled
}

}  // namespace rumblex_movement
