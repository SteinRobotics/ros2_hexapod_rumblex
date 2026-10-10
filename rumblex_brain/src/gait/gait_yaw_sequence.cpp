#include "gait/gait_yaw_sequence.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "gait/trajectory.hpp"
#include "movement_request.hpp"

namespace brain {

CYawSequenceGait::CYawSequenceGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> kinematics,
                                   Sweep sweep, units::Angle head_amplitude, units::Angle torso_amplitude)
    : node_(std::move(node)),
      kinematics_(std::move(kinematics)),
      phase_limit_((sweep == Sweep::OneSide ? M_PI : 2.0 * M_PI) * units::rad),
      head_amplitude_(head_amplitude),
      torso_amplitude_(torso_amplitude) {
}

void CYawSequenceGait::start(units::Duration duration_s, uint8_t direction) {
    RCLCPP_INFO(node_->get_logger(), "Starting yaw sequence");
    state_ = EGaitState::Running;
    phase_ = 0.0 * units::rad;
    torso_origin_ = kinematics_->getTorsoPose();
    head_origin_ = kinematics_->getHeadOrientation();
    toe_origins_ = kinematics_->getToePositions();
    direction_sign_ = direction == MovementRequest::CLOCKWISE ? -1.0 : 1.0;
    phase_rate_ = phase_limit_ / std::max(duration_s, 1.0 * units::s);
}

bool CYawSequenceGait::update() {
    return updateTimed(0.1 * units::s);
}

bool CYawSequenceGait::updateTimed(units::Duration elapsed_s) {
    if (!mp_units::isfinite(elapsed_s) || elapsed_s <= 0.0 * units::s || elapsed_s > 0.5 * units::s)
        return false;
    if (state_ == EGaitState::Stopped) return false;

    phase_ = std::min<units::Angle>(phase_ + phase_rate_ * elapsed_s, phase_limit_);
    if ((phase_ / phase_limit_).numerical_value_in(mp_units::one) >= 1.0 - 1e-12) phase_ = phase_limit_;
    const auto angle =
        phase_limit_ * trajectoryProgress((phase_ / phase_limit_).numerical_value_in(mp_units::one));
    const double excursion =
        phase_ >= phase_limit_
            ? 0.0
            : direction_sign_ * mp_units::angular::sin(angle).numerical_value_in(mp_units::one);
    auto head = head_origin_;
    head.yaw += head_amplitude_ * excursion;
    kinematics_->setHeadOrientation(head);
    auto torso = torso_origin_;
    torso.orientation.yaw += torso_amplitude_ * excursion;
    kinematics_->moveTorso(toe_origins_, torso);
    if (phase_ >= phase_limit_) state_ = EGaitState::Stopped;
    return true;
}

}  // namespace brain
