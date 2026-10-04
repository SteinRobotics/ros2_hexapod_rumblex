#include "gait/gait_continuous_pose.hpp"

#include "gait/trajectory.hpp"

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

    tick_ = 10;
    toe_origins_ = kinematics_->getToePositions();
    torso_origin_ = kinematics_->getTorsoPose();
    head_origin_ = kinematics_->getHeadOrientation();
    torso_target_ = torso_origin_;
    head_target_ = head_origin_;
}

bool CContinuousPoseGait::update(const geometry_msgs::msg::Twist& /*velocity*/, const CPose& torso,
                                 const COrientation& head) {
    if (state_ == EGaitState::Stopped) return false;

    // Finish each finite pose trajectory before accepting another target or stop.
    if (tick_ == 10) {
        if (state_ == EGaitState::Stopping) {
            state_ = EGaitState::Stopped;
            return false;
        }
        torso_origin_ = kinematics_->getTorsoPose();
        head_origin_ = kinematics_->getHeadOrientation();
        if (torso_origin_ == torso && head_origin_ == head) return false;
        torso_target_ = torso;
        head_target_ = head;
        tick_ = 0;
    }
    const double progress = trajectoryProgress(static_cast<double>(++tick_) / 10.0);
    kinematics_->moveTorso(
        toe_origins_, tick_ == 10 ? torso_target_ : torso_origin_.linearInterpolate(torso_target_, progress));
    kinematics_->setHeadOrientation(tick_ == 10 ? head_target_
                                                : head_origin_.linearInterpolate(head_target_, progress));
    if (tick_ == 10 && state_ == EGaitState::Stopping) state_ = EGaitState::Stopped;
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
