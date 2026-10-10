#include "gait/gait_single_pose.hpp"

#include <algorithm>
#include <cmath>

#include "gait/trajectory.hpp"

using namespace rumblex_interfaces::msg;
namespace brain {

CSinglePoseGait::CSinglePoseGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> kinematics,
                                 Parameters::SinglePose& /*params*/, std::optional<Target> target)
    : node_(node), kinematics_(kinematics), target_(std::move(target)) {
}

void CSinglePoseGait::start(units::Duration duration_s, uint8_t /*direction*/) {
    RCLCPP_INFO(node_->get_logger(), "Starting CSinglePoseGait");
    state_ = EGaitState::Running;
    phase_ = 0.0 * units::s;
    duration_s_ = std::max(duration_s, 1.0 * units::s);

    torso_origin_ = kinematics_->getTorsoPose();
    head_origin_ = kinematics_->getHeadOrientation();

    toe_origins_ = kinematics_->getToePositions();
    if (target_ && toe_origins_ == target_->toes && torso_origin_ == target_->torso &&
        head_origin_ == target_->head) {
        state_ = EGaitState::Stopped;
    }
}

bool CSinglePoseGait::update(const Velocity& velocity, const CPose& torso, const COrientation& head) {
    return updateTimed(velocity, torso, head, 0.1 * units::s);
}

bool CSinglePoseGait::updateTimed(const Velocity&, const CPose& torso, const COrientation& head,
                                  units::Duration elapsed_s) {
    if (!mp_units::isfinite(elapsed_s) || elapsed_s <= 0.0 * units::s || elapsed_s > 0.5 * units::s)
        return false;
    if (state_ == EGaitState::Stopped) return false;

    if (phase_ == 0.0 * units::s) {
        torso_target_ = target_ ? target_->torso : torso;
        head_target_ = target_ ? target_->head : head;
    }
    phase_ += elapsed_s;
    const double fraction = duration_s_ > 0.0 * units::s
                                ? std::min((phase_ / duration_s_).numerical_value_in(mp_units::one), 1.0)
                                : 1.0;
    if (fraction >= 1.0 - 1e-12) state_ = EGaitState::Stopped;
    double progress = state_ == EGaitState::Stopped ? 1.0 : fraction;
    progress = trajectoryProgress(progress);
    const auto& target_torso = torso_target_;
    const auto& target_head = head_target_;
    const CPose intermediate_pose = state_ == EGaitState::Stopped
                                        ? target_torso
                                        : torso_origin_.linearInterpolate(target_torso, progress);
    if (target_) {
        std::map<ELegIndex, CPosition> toes;
        for (const auto& [index, position] : target_->toes) {
            toes[index] = state_ == EGaitState::Stopped
                              ? position
                              : toe_origins_.at(index).linearInterpolate(position, progress);
        }
        kinematics_->moveTorso(toes, intermediate_pose);
    } else {
        kinematics_->moveTorso(toe_origins_, intermediate_pose);
    }
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
