#include "gait/gait_leg_wave.hpp"

#include <magic_enum/magic_enum.hpp>

#include "gait/pose_model.hpp"
#include "gait/trajectory.hpp"

namespace brain {

CLegWaveGait::CLegWaveGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> kinematics,
                           Parameters::LegWave& params)
    : node_(node), kinematics_(kinematics), params_(params) {
}

void CLegWaveGait::start(double /*duration_s*/, uint8_t direction) {
    state_ = EGaitState::Running;
    phase_ = 0.0;
    origins_ = kinematics_->getToePositions();
    direction_ = direction;
    active_leg_index_ = ELegIndex::RightFront;
}

bool CLegWaveGait::update() {
    if (state_ == EGaitState::Stopped) {
        return false;
    }
    constexpr double delta_phase = M_PI / 10.0;
    phase_ = std::min(phase_ + delta_phase, M_PI);
    const auto& base_toe_pos = origins_;

    if (phase_ >= M_PI) {
        // reset last leg to neutral position
        kinematics_->setToePosition(active_leg_index_, base_toe_pos.at(active_leg_index_));

        if (state_ == EGaitState::Stopping) {
            state_ = EGaitState::Stopped;
            return true;
        }
        // advance to the next leg
        size_t step = 1;
        if (direction_ != 0) {
            step = leg_order_.size() - 1;
        }
        active_leg_index_ = leg_order_[(std::find(leg_order_.begin(), leg_order_.end(), active_leg_index_) -
                                        leg_order_.begin() + step) %
                                       leg_order_.size()];
        phase_ = 0.0;
    }

    auto target_position = base_toe_pos.at(active_leg_index_);
    target_position.z =
        base_toe_pos.at(active_leg_index_).z + params_.leg_lift_height * trajectoryLift(phase_ / M_PI);
    RCLCPP_DEBUG_STREAM(node_->get_logger(),
                        "LegWave: Moving leg " << magic_enum::enum_name(active_leg_index_) << " to position ("
                                               << target_position.x << ", " << target_position.y << ", "
                                               << target_position.z << ") at phase " << phase_);

    kinematics_->setToePosition(active_leg_index_, target_position);
    return true;
}

void CLegWaveGait::requestStop() {
    if (state_ == EGaitState::Running) {
        state_ = EGaitState::Stopping;
    }
}

void CLegWaveGait::cancelStop() {
    if (state_ == EGaitState::Stopping) {
        state_ = EGaitState::Running;
    }
}

}  // namespace brain
