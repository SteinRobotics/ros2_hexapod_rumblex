#include "gait/gait_single_pose.hpp"

#include <algorithm>
#include <cmath>

#include "gait/trajectory.hpp"

using namespace rumblex_interfaces::msg;
namespace brain {

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using mp_units::angular::unit_symbols::deg;

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
    duration_s_ = delta_ ? duration_s : std::max(duration_s, 1.0 * s);

    torso_origin_ = kinematics_->getTorsoPose();
    head_origin_ = kinematics_->getHeadOrientation();

    toe_origins_ = kinematics_->getToePositions();
    const auto origin = bodyPose(*kinematics_);
    if (options_.save_pose) saved_pose_ = origin;
    auto target = resolveBodyPose(origin, target_);
    if (options_.restore_pose) {
        if (!saved_pose_) {
            state_ = EGaitState::Stopped;
            return;
        }
        target = *saved_pose_;
        saved_pose_.reset();
    }
    if (options_.hasJointTargets()) {
        auto model = *kinematics_;
        model.moveTorso(toeTargets(target), CPose(target.torso_pose));
        model.setHeadOrientation(COrientation(target.head_orientation));
        size_t i = 0;
        for (const auto& [leg, toe] : toe_origins_) {
            const auto& angles = options_.joint_targets[i++];
            if (angles) model.setLegAngles(leg, CLegAngles((*angles)[0], (*angles)[1], (*angles)[2]));
        }
        target = bodyPose(model);
    }
    if (delta_) {
        // Work on a copy: computing FK targets must not change the live start pose.
        auto model = *kinematics_;
        auto torso = torso_origin_;
        torso.orientation.roll += delta_->torso_rotation.roll * deg;
        torso.orientation.pitch += delta_->torso_rotation.pitch * deg;
        torso.orientation.yaw += delta_->torso_rotation.yaw * deg;
        auto head = head_origin_;
        head.roll += delta_->head_rotation.roll * deg;
        head.pitch += delta_->head_rotation.pitch * deg;
        head.yaw += delta_->head_rotation.yaw * deg;
        model.setHeadOrientation(head);
        torso.position = torso.position + CPosition(delta_->torso_translation.x, delta_->torso_translation.y,
                                                    delta_->torso_translation.z);
        auto toes = toe_origins_;
        size_t i = 0;
        for (auto& [leg, toe] : toes) {
            const auto& offset = delta_->toe_translation[i++];
            toe = toe + CPosition(offset.x, offset.y, offset.z);
        }
        model.moveTorso(toes, torso);
        i = 0;
        for (const auto& [leg, toe] : toes) {
            const double rotation = delta_->coxa_rotation_deg[i++];
            if (rotation == 0.0) continue;
            auto angles = model.getLegAngles(leg);
            angles.torso_coxa += rotation * deg;
            model.setLegAngles(leg, angles);
        }
        target = bodyPose(model);
    }
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
