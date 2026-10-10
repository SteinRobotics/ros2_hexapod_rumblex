/*******************************************************************************
 * Copyright (c) 2024 Christian Stein
 ******************************************************************************/

#pragma once
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <magic_enum/magic_enum.hpp>
#include <map>
#include <memory>
#include <string>

#include "gait/gait_clap.hpp"
#include "gait/gait_continuous_pose.hpp"
#include "gait/gait_high_five.hpp"
#include "gait/gait_interfaces.hpp"
#include "gait/gait_leg_wave.hpp"
#include "gait/gait_move_combined.hpp"
#include "gait/gait_parameters.hpp"
#include "gait/gait_running.hpp"
#include "gait/gait_single_pose.hpp"
#include "gait/gait_test_legs.hpp"
#include "gait/gait_torso_roll.hpp"
#include "gait/gait_waiting.hpp"
#include "gait/gait_yaw_sequence.hpp"
#include "gait/pose_model.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "movement_request.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rumblex_utils/geometry.hpp"

namespace brain {

class CGaitController {
   public:
    CGaitController(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> kinematics);
    virtual ~CGaitController();

    using MovementRequestType = brain::MovementRequest::_type_type;

    std::function<void(const MovementRequest&)> on_gait_changed;
    bool moving() const {
        return (currentGait() == MovementRequest::CONTINUOUS_MOVE ||
                currentGait() == MovementRequest::CONTINUOUS_RUNNING) &&
               active_gait_->state() != EGaitState::Stopped;
    }
    bool setGait(brain::MovementRequest request);
    bool hasPendingGait() const {
        return pending_request_.type != MovementRequest::NO_REQUEST;
    }
    bool stopped() const {
        return active_gait_->state() == EGaitState::Stopped;
    }
    bool finishesAutomatically() const;
    MovementRequestType currentGait() const {
        return active_request_.type;
    }

    bool updateSelectedGait(const Velocity& velocity, CPose torso = CPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                            COrientation head = COrientation(0.0, 0.0, 0.0),
                            units::Duration elapsed_s = 0.1 * units::s);
    void requestStopSelectedGait();

   private:
    void switchGait(brain::MovementRequest request);
    static brain::MovementRequest createMsg(std::string name, MovementRequestType type);

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CPoseModel> kinematics_;
    Parameters params_;

    std::map<MovementRequestType, std::shared_ptr<brain::IGait>> gaits_;

    std::shared_ptr<brain::IGait> active_gait_ = nullptr;

    brain::MovementRequest active_request_;
    brain::MovementRequest pending_request_;
};

}  // namespace brain