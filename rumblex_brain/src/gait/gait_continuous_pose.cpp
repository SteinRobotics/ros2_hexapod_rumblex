#include "gait/gait_continuous_pose.hpp"

using namespace rumblex_interfaces::msg;
namespace brain {

CContinuousPoseGait::CContinuousPoseGait(std::shared_ptr<rclcpp::Node> node,
                                         std::shared_ptr<CPoseModel> kinematics,
                                         Parameters::ContinuousPose& params)
    : node_(node), kinematics_(kinematics), params_(params) {
}

void CContinuousPoseGait::start(double /*duration_s*/, uint8_t /*direction*/) {
    RCLCPP_INFO(node_->get_logger(), "Starting CContinuousPoseGait");
    state_ = EGaitState::Running;

    torso_origin_ = kinematics_->getTorsoPose();
    head_origin_ = kinematics_->getHeadOrientation();
    torso_target_ = torso_origin_;
    head_target_ = head_origin_;
}

bool CContinuousPoseGait::update(const geometry_msgs::msg::Twist& /*velocity*/, const CPose& torso,
                                 const COrientation& head) {
    if (state_ == EGaitState::Stopped) return false;

    if (state_ == EGaitState::Stopping) {
        kinematics_->moveTorso(torso_origin_);
        kinematics_->setHeadOrientation(head_origin_);
        state_ = EGaitState::Stopped;
        return true;
    }

    torso_target_ = torso_target_.linearInterpolate(torso, 0.2);
    head_target_ = head_target_.linearInterpolate(head, 0.5);

    kinematics_->moveTorso(torso_target_);
    kinematics_->setHeadOrientation(head_target_);
    return true;
}

void CContinuousPoseGait::requestStop() {
    if (state_ == EGaitState::Running) {
        state_ = EGaitState::Stopping;
    }
}

void CContinuousPoseGait::cancelStop() {
    if (state_ == EGaitState::Stopping) {
        state_ = EGaitState::Running;
    }
}

}  // namespace brain
