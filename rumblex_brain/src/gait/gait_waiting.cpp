#include "gait/gait_waiting.hpp"

#include "gait/pose_model.hpp"
#include "gait/trajectory.hpp"

namespace brain {
CWaitingGait::CWaitingGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> kinematics,
                           Parameters::Waiting& params)
    : node_(node), kinematics_(kinematics), params_(params) {
}

void CWaitingGait::start(units::Duration /*duration_s*/, uint8_t /*direction*/) {
    // Directly enter Running; no distinct Starting phase needed.
    state_ = EGaitState::Running;
    phase_ = 0.0 * units::rad;
    torso_origin_ = kinematics_->getTorsoPose();
    toe_origins_ = kinematics_->getToePositions();
}

bool CWaitingGait::update() {
    if (state_ == EGaitState::Stopped) {
        return false;
    }
    phase_ = std::min<units::Angle>(phase_ + 0.1 * units::rad, units::Angle(2.0 * M_PI * units::rad));
    auto torso_target = torso_origin_;
    const double excursion =
        phase_ >= 2.0 * M_PI * units::rad
            ? 0.0
            : mp_units::angular::sin(
                  2.0 * M_PI * units::rad *
                  trajectoryProgress((phase_ / (2.0 * M_PI * units::rad)).numerical_value_in(mp_units::one)))
                  .numerical_value_in(mp_units::one);
    torso_target.position.z += 0.05 * units::m * excursion;
    kinematics_->moveTorso(toe_origins_, torso_target);
    if (phase_ >= 2.0 * M_PI * units::rad) {
        phase_ = 0.0 * units::rad;
        if (state_ == EGaitState::Stopping) state_ = EGaitState::Stopped;
    }
    return true;
}

void CWaitingGait::requestStop() {
    if (state_ == EGaitState::Running) {
        state_ = EGaitState::Stopping;
    }
}

void CWaitingGait::cancelStop() {
    if (state_ == EGaitState::Stopping) {
        state_ = EGaitState::Running;
    }
}

}  // namespace brain
