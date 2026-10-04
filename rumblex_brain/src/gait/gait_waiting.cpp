#include "gait/gait_waiting.hpp"

#include "gait/pose_model.hpp"
#include "gait/trajectory.hpp"

namespace brain {
CWaitingGait::CWaitingGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> kinematics,
                           Parameters::Waiting& params)
    : node_(node), kinematics_(kinematics), params_(params) {
}

void CWaitingGait::start(double /*duration_s*/, uint8_t /*direction*/) {
    // Directly enter Running; no distinct Starting phase needed.
    state_ = EGaitState::Running;
    phase_ = 0.0;
    torso_origin_ = kinematics_->getTorsoPose();
    toe_origins_ = kinematics_->getToePositions();
}

bool CWaitingGait::update() {
    if (state_ == EGaitState::Stopped) {
        return false;
    }
    phase_ = std::min(phase_ + 0.1, 2.0 * M_PI);
    auto torso_target = torso_origin_;
    const double excursion =
        phase_ >= 2.0 * M_PI ? 0.0 : std::sin(2.0 * M_PI * trajectoryProgress(phase_ / (2.0 * M_PI)));
    torso_target.position.z += 0.05 * units::m * excursion;
    kinematics_->moveTorso(toe_origins_, torso_target);
    if (phase_ >= 2.0 * M_PI) {
        phase_ = 0.0;
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
