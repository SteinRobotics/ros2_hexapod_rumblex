#include "gait/gait_watch.hpp"

#include <cmath>

using namespace rumblex_interfaces::msg;
namespace brain {

CWatchGait::CWatchGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> kinematics,
                       Parameters::Watch& params)
    : node_(node), kinematics_(kinematics), params_(params) {
}

void CWatchGait::start(double duration_s, uint8_t direction) {
    RCLCPP_INFO(node_->get_logger(), "Starting CWatchGait");
    state_ = EGaitState::Running;
    phase_ = 0.0;

    amplitude_head_ =
        (direction == MovementRequest::CLOCKWISE) ? params_.head_max_yaw : -params_.head_max_yaw;
    amplitude_torso_ =
        (direction == MovementRequest::CLOCKWISE) ? params_.torso_max_yaw : -params_.torso_max_yaw;

    // 100ms task update time, duration in seconds, 1 full cycle = 2pi
    delta_phase_ = (2.0 * M_PI) / (duration_s / 0.1);
}

bool CWatchGait::update() {
    if (state_ == EGaitState::Stopped) return false;

    // set head yaw using sinusoidal oscillation
    phase_ += delta_phase_;
    if (phase_ > 2.0 * M_PI) {
        state_ = EGaitState::Stopped;
        kinematics_->setHeadOrientation(COrientation(0.0, 0.0, 0.0));
        kinematics_->moveTorso(CPose());
        return true;
    }
    COrientation head_request;
    head_request.yaw = amplitude_head_ * std::sin(phase_);
    kinematics_->setHeadOrientation(head_request);

    // CPose torso_request;
    // torso_request.orientation.yaw = amplitude_torso_ * std::sin(phase_);
    // kinematics_->moveTorso(torso_request);

    return true;
}

void CWatchGait::requestStop() {
    // gait is stopped automatically after completing the current cycle
}

void CWatchGait::cancelStop() {
    // this gait cannot be stopped nor the stop can be cancelled
}

}  // namespace brain
