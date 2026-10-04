#include "gait/gait_watch.hpp"

#include <cmath>

#include "gait/trajectory.hpp"

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
    torso_origin_ = kinematics_->getTorsoPose();
    head_origin_ = kinematics_->getHeadOrientation();
    toe_origins_ = kinematics_->getToePositions();

    amplitude_head_ =
        (direction == MovementRequest::CLOCKWISE) ? params_.head_max_yaw : -params_.head_max_yaw;
    amplitude_torso_ =
        (direction == MovementRequest::CLOCKWISE) ? params_.torso_max_yaw : -params_.torso_max_yaw;

    // 100ms task update time, duration in seconds, 1 full cycle = 2pi
    delta_phase_ = (2.0 * M_PI) / (std::max(duration_s, 1.0) / 0.1);
}

bool CWatchGait::update() {
    if (state_ == EGaitState::Stopped) return false;

    phase_ = std::min(phase_ + delta_phase_, 2.0 * M_PI);
    const double angle = (2.0 * M_PI) * trajectoryProgress(phase_ / (2.0 * M_PI));
    const double excursion = phase_ >= 2.0 * M_PI ? 0.0 : std::sin(angle);
    auto head_request = head_origin_;
    head_request.yaw += amplitude_head_ * excursion;
    kinematics_->setHeadOrientation(head_request);
    auto torso_request = torso_origin_;
    kinematics_->moveTorso(toe_origins_, torso_request);
    if (phase_ >= 2.0 * M_PI) state_ = EGaitState::Stopped;

    return true;
}

void CWatchGait::requestStop() {
    // gait is stopped automatically after completing the current cycle
}

void CWatchGait::cancelStop() {
    // this gait cannot be stopped nor the stop can be cancelled
}

}  // namespace brain
