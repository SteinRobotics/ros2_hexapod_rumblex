#include "requester/gait_look.hpp"

#include <cmath>

using namespace rumblex_interfaces::msg;
namespace rumblex_movement {

CGaitLook::CGaitLook(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                     Parameters::Look& params)
    : node_(node), kinematics_(kinematics), params_(params) {
}

void CGaitLook::start(double duration_s, uint8_t direction) {
    RCLCPP_INFO(node_->get_logger(), "Starting CGaitLook");
    state_ = EGaitState::Running;
    phase_ = 0.0;

    amplitude_head_deg_ =
        (direction == MovementRequest::CLOCKWISE) ? params_.head_max_yaw_deg : -params_.head_max_yaw_deg;
    amplitude_body_deg_ =
        (direction == MovementRequest::CLOCKWISE) ? params_.body_max_yaw_deg : -params_.body_max_yaw_deg;

    // 100ms task update time, duration in seconds, 1 full cycle = 2pi
    delta_phase_ = (M_PI) / (duration_s / 0.1);
}

bool CGaitLook::update() {
    if (state_ == EGaitState::Stopped) return false;

    // set head yaw using sinusoidal oscillation
    phase_ += delta_phase_;
    if (phase_ > M_PI) {
        state_ = EGaitState::Stopped;
        kinematics_->setHead(COrientation(0.0, 0.0, 0.0));
        kinematics_->moveBody(CPose());
        return true;
    }
    COrientation head_request;
    head_request.yaw_deg = amplitude_head_deg_ * std::sin(phase_);
    kinematics_->setHead(head_request);

    CPose body_request;
    body_request.orientation.yaw_deg = amplitude_body_deg_ * std::sin(phase_);
    kinematics_->moveBody(body_request);

    return true;
}

void CGaitLook::requestStop() {
    // gait is stopped automatically after completing the current cycle
}

void CGaitLook::cancelStop() {
    // this gait cannot be stopped nor the stop can be cancelled
}

}  // namespace rumblex_movement
