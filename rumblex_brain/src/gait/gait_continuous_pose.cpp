#include "gait/gait_continuous_pose.hpp"

#include "gait/trajectory.hpp"

using namespace rumblex_interfaces::msg;
namespace brain {

using namespace mp_units;
using namespace mp_units::si::unit_symbols;

CContinuousPoseGait::CContinuousPoseGait(std::shared_ptr<rclcpp::Node> node,
                                         std::shared_ptr<CPoseModel> kinematics,
                                         Parameters::ContinuousPose& params)
    : node_(node), kinematics_(kinematics), params_(params) {
}

void CContinuousPoseGait::start(units::Duration /*duration_s*/, uint8_t /*direction*/) {
    RCLCPP_INFO(node_->get_logger(), "Starting CContinuousPoseGait");
    state_ = EGaitState::Running;

    elapsed_s_ = 1.0 * s;
    toe_origins_ = kinematics_->getToePositions();
    torso_origin_ = kinematics_->getTorsoPose();
    head_origin_ = kinematics_->getHeadOrientation();
    torso_target_ = torso_origin_;
    head_target_ = head_origin_;
}

bool CContinuousPoseGait::update(const Velocity& velocity, const CPose& torso, const COrientation& head) {
    return updateTimed(velocity, torso, head, 0.1 * s);
}

bool CContinuousPoseGait::updateTimed(const Velocity&, const CPose& torso, const COrientation& head,
                                      units::Duration elapsed_s) {
    if (!isfinite(elapsed_s) || elapsed_s <= 0.0 * s || elapsed_s > 0.5 * s) return false;
    if (state_ == EGaitState::Stopped) return false;

    // Finish each finite pose trajectory before accepting another target or stop.
    if (elapsed_s_ >= 1.0 * s) {
        if (state_ == EGaitState::Stopping) {
            state_ = EGaitState::Stopped;
            return false;
        }
        torso_origin_ = kinematics_->getTorsoPose();
        head_origin_ = kinematics_->getHeadOrientation();
        if (torso_origin_ == torso && head_origin_ == head) return false;
        torso_target_ = torso;
        head_target_ = head;
        elapsed_s_ = 0.0 * s;
    }
    elapsed_s_ = std::min(elapsed_s_ + elapsed_s, 1.0 * s);
    if (elapsed_s_ >= (1.0 - 1e-12) * s) elapsed_s_ = 1.0 * s;
    const double progress = trajectoryProgress((elapsed_s_ / (1.0 * s)).numerical_value_in(one));
    // Reach the head target sooner while retaining a rest-to-rest trajectory.
    constexpr auto head_duration = 0.5 * s;
    const double head_progress = trajectoryProgress((elapsed_s_ / head_duration).numerical_value_in(one));
    kinematics_->moveTorso(toe_origins_, elapsed_s_ >= 1.0 * s
                                             ? torso_target_
                                             : torso_origin_.linearInterpolate(torso_target_, progress));
    kinematics_->setHeadOrientation(elapsed_s_ >= head_duration
                                        ? head_target_
                                        : head_origin_.linearInterpolate(head_target_, head_progress));
    if (elapsed_s_ >= 1.0 * s && state_ == EGaitState::Stopping) state_ = EGaitState::Stopped;
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
