#include "requester/gait_continuouspose.hpp"

using namespace rumblex_interfaces::msg;
namespace rumblex_movement {

CGaitContinuousPose::CGaitContinuousPose(std::shared_ptr<rclcpp::Node> node,
                                         std::shared_ptr<CKinematics> kinematics,
                                         Parameters::ContinuousPose& params)
    : node_(node), kinematics_(kinematics), params_(params) {
}

void CGaitContinuousPose::start(double /*duration_s*/, uint8_t /*direction*/) {
    RCLCPP_INFO(node_->get_logger(), "Starting CGaitContinuousPose");
    state_ = EGaitState::Running;

    body_origin_ = kinematics_->getBody();
    head_origin_ = kinematics_->getHead();
    body_target_ = body_origin_;
    head_target_ = head_origin_;
}

bool CGaitContinuousPose::update(const geometry_msgs::msg::Twist& /*velocity*/, const CPose& body,
                                 const COrientation& head) {
    if (state_ == EGaitState::Stopped) return false;

    if (state_ == EGaitState::Stopping) {
        kinematics_->moveBody(body_origin_);
        kinematics_->setHead(head_origin_);
        state_ = EGaitState::Stopped;
        return true;
    }

    body_target_ = body_target_.linearInterpolate(body, 0.2);
    head_target_ = head_target_.linearInterpolate(head, 0.5);

    kinematics_->moveBody(body_target_);
    kinematics_->setHead(head_target_);
    return true;
}

void CGaitContinuousPose::requestStop() {
    if (state_ == EGaitState::Running) {
        state_ = EGaitState::Stopping;
    }
}

void CGaitContinuousPose::cancelStop() {
    // this gait cannot be stopped nor the stop can be cancelled
}

}  // namespace rumblex_movement
