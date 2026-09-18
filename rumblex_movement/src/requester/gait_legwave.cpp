#include "requester/gait_legwave.hpp"

#include <magic_enum.hpp>

#include "requester/kinematics.hpp"

namespace rumblex_movement {

CGaitLegWave::CGaitLegWave(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                           Parameters::LegWave& params)
    : node_(node), kinematics_(kinematics), params_(params) {
}

void CGaitLegWave::start(double /*duration_s*/, uint8_t direction) {
    state_ = EGaitState::Running;
    phase_ = 0.0;
    direction_ = direction;
    active_leg_index_ = ELegIndex::RightFront;
}

bool CGaitLegWave::update() {
    if (state_ == EGaitState::Stopped) {
        return false;
    }
    // TODO: calc velocity_mag to delta_phase
    constexpr double delta_phase = 0.5;
    phase_ += delta_phase;

    if (state_ == EGaitState::Stopping && utils::isSinValueNearZero(phase_, delta_phase)) {
        // Reset the active leg to standing position before stopping
        const auto base_foot_pos = kinematics_->getLegsStandingPositions();
        kinematics_->setSingleFeet(active_leg_index_, base_foot_pos.at(active_leg_index_));
        state_ = EGaitState::Stopped;
        return false;
    }

    // TODO better use kinematics_->getLegsPositions()
    const auto base_foot_pos = kinematics_->getLegsStandingPositions();

    if (phase_ >= M_PI) {
        // reset last leg to neutral position
        kinematics_->setSingleFeet(active_leg_index_, base_foot_pos.at(active_leg_index_));

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

    auto target_position = base_foot_pos.at(active_leg_index_);
    target_position.z = base_foot_pos.at(active_leg_index_).z + params_.leg_lift_height * std::sin(phase_);
    RCLCPP_DEBUG_STREAM(node_->get_logger(),
                        "LegWave: Moving leg " << magic_enum::enum_name(active_leg_index_) << " to position ("
                                               << target_position.x << ", " << target_position.y << ", "
                                               << target_position.z << ") at phase " << phase_);

    kinematics_->setSingleFeet(active_leg_index_, target_position);
    return true;
}

void CGaitLegWave::requestStop() {
    if (state_ == EGaitState::Running) {
        state_ = EGaitState::Stopping;
    }
}

void CGaitLegWave::cancelStop() {
    if (state_ == EGaitState::Stopping) {
        state_ = EGaitState::Running;
    }
}

}  // namespace rumblex_movement
