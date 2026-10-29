#include "gait/gait_single_pose.hpp"

#include <algorithm>
#include <cmath>

#include "gait/trajectory.hpp"

using namespace rumblex_interfaces::msg;
namespace brain {

using namespace mp_units;
using namespace mp_units::si::unit_symbols;

CSinglePoseGait::CSinglePoseGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> kinematics,
                                 Parameters::SinglePose& /*params*/, rumblex_interfaces::msg::BodyPose target)
    : node_(node),
      kinematics_(kinematics),
      target_(std::move(target)),
      skip_if_at_target_(completeBodyPose(target_)) {
}

void CSinglePoseGait::start(units::Duration duration_s, uint8_t /*direction*/) {
    RCLCPP_INFO(node_->get_logger(), "Starting CSinglePoseGait");
    state_ = EGaitState::Running;
    phase_ = 0.0 * s;
    duration_s_ = std::max(duration_s, 1.0 * s);

    torso_origin_ = kinematics_->getTorsoPose();
    head_origin_ = kinematics_->getHeadOrientation();

    toe_origins_ = kinematics_->getToePositions();
    const auto target = resolveBodyPose(bodyPose(*kinematics_), target_);
    torso_target_ = CPose(target.torso_pose);
    head_target_ = COrientation(target.head_orientation);
    toe_targets_ = toeTargets(target);
    // Stand-up/laydown can complete immediately when already at their preset.
    // Ordinary pose actions still consume their duration, including repeated holds.
    if (skip_if_at_target_ && toe_origins_ == toe_targets_ && torso_origin_ == torso_target_ &&
        head_origin_ == head_target_) {
        state_ = EGaitState::Stopped;
    }
}

bool CSinglePoseGait::update(const Velocity& velocity, const CPose& torso, const COrientation& head) {
    return updateTimed(velocity, torso, head, 0.1 * s);
}

bool CSinglePoseGait::updateTimed(const Velocity&, const CPose&, const COrientation&,
                                  units::Duration elapsed_s) {
    if (!isfinite(elapsed_s) || elapsed_s <= 0.0 * s || elapsed_s > 0.5 * s) return false;
    if (state_ == EGaitState::Stopped) return false;

    phase_ += elapsed_s;
    const double fraction =
        duration_s_ > 0.0 * s ? std::min((phase_ / duration_s_).numerical_value_in(one), 1.0) : 1.0;
    if (fraction >= 1.0 - 1e-12) state_ = EGaitState::Stopped;
    double progress = state_ == EGaitState::Stopped ? 1.0 : fraction;
    progress = trajectoryProgress(progress);
    const auto& target_torso = torso_target_;
    const auto& target_head = head_target_;
    const CPose intermediate_pose = state_ == EGaitState::Stopped
                                        ? target_torso
                                        : torso_origin_.linearInterpolate(target_torso, progress);
    std::map<ELegIndex, CPosition> toes;
    for (const auto& [index, position] : toe_targets_) {
        toes[index] = state_ == EGaitState::Stopped
                          ? position
                          : toe_origins_.at(index).linearInterpolate(position, progress);
    }
    kinematics_->moveTorso(toes, intermediate_pose);
    kinematics_->setHeadOrientation(
        state_ == EGaitState::Stopped ? target_head : head_origin_.linearInterpolate(target_head, progress));

    return true;
}

void CSinglePoseGait::requestStop() {
    // gait is stopped automatically after completing the current cycle
}

void CSinglePoseGait::cancelStop() {
    // this gait cannot be stopped nor the stop can be cancelled
}

}  // namespace brain
