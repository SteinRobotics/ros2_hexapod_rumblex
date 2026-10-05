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

    elapsed_s_ = 1.0;
    toe_origins_ = kinematics_->getToePositions();
    torso_origin_ = kinematics_->getTorsoPose();
    head_origin_ = kinematics_->getHeadOrientation();
    torso_target_ = torso_origin_;
    head_target_ = head_origin_;
}

bool CContinuousPoseGait::update(const geometry_msgs::msg::Twist& velocity, const CPose& torso,
                                 const COrientation& head) {
    return updateTimed(velocity, torso, head, 0.1);
}

bool CContinuousPoseGait::updateTimed(const geometry_msgs::msg::Twist&, const CPose& torso,
                                      const COrientation& head, double elapsed_s) {
    if (!std::isfinite(elapsed_s) || elapsed_s <= 0.0 || elapsed_s > 0.5) return false;
    if (state_ == EGaitState::Stopped) return false;

    // Finish each finite pose trajectory before accepting another target or stop.
    if (elapsed_s_ >= 1.0) {
        if (state_ == EGaitState::Stopping) {
            state_ = EGaitState::Stopped;
            return false;
        }
        torso_origin_ = kinematics_->getTorsoPose();
        head_origin_ = kinematics_->getHeadOrientation();
        if (torso_origin_ == torso && head_origin_ == head) return false;
        torso_target_ = torso;
        head_target_ = head;
        elapsed_s_ = 0.0;
    }
    elapsed_s_ = std::min(elapsed_s_ + elapsed_s, 1.0);
    if (elapsed_s_ >= 1.0 - 1e-12) elapsed_s_ = 1.0;
    const double progress = trajectoryProgress(elapsed_s_);
    kinematics_->moveTorso(toe_origins_, elapsed_s_ >= 1.0
                                             ? torso_target_
                                             : torso_origin_.linearInterpolate(torso_target_, progress));
    kinematics_->setHeadOrientation(
        elapsed_s_ >= 1.0 ? head_target_ : head_origin_.linearInterpolate(head_target_, progress));
    if (elapsed_s_ >= 1.0 && state_ == EGaitState::Stopping) state_ = EGaitState::Stopped;
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
