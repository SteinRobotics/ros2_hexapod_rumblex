/*******************************************************************************
 * Copyright (c) 2024 Christian Stein
 ******************************************************************************/

#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <magic_enum.hpp>
#include <map>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "requester/gait_clap.hpp"
#include "requester/gait_continuous_pose.hpp"
#include "requester/gait_high_five.hpp"
#include "requester/gait_interfaces.hpp"
#include "requester/gait_lay_down.hpp"
#include "requester/gait_leg_wave.hpp"
#include "requester/gait_look.hpp"
#include "requester/gait_move_combined.hpp"
#include "requester/gait_parameters.hpp"
#include "requester/gait_running.hpp"
#include "requester/gait_single_pose.hpp"
#include "requester/gait_stand_up.hpp"
#include "requester/gait_test_legs.hpp"
#include "requester/gait_torso_roll.hpp"
#include "requester/gait_waiting.hpp"
#include "requester/gait_watch.hpp"
#include "requester/kinematics.hpp"
#include "requester/types.hpp"
#include "rumblex_interfaces/msg/movement_request.hpp"
#include "rumblex_utils/geometry.hpp"

namespace rumblex_movement {

class CGaitController {
   public:
    CGaitController(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics);
    virtual ~CGaitController();

    using MovementRequestType = rumblex_interfaces::msg::MovementRequest::_type_type;

    void setGait(rumblex_interfaces::msg::MovementRequest request);
    MovementRequestType currentGait() const {
        return active_request_.type;
    }

    bool updateSelectedGait(const geometry_msgs::msg::Twist& velocity,
                            CPose torso = CPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                            COrientation head = COrientation(0.0, 0.0, 0.0));
    void requestStopSelectedGait();

   private:
    void switchGait(rumblex_interfaces::msg::MovementRequest request);
    static rumblex_interfaces::msg::MovementRequest createMsg(std::string name, MovementRequestType type);

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;
    Parameters params_;

    std::map<MovementRequestType, std::shared_ptr<rumblex_movement::IGait>> gaits_;

    std::shared_ptr<rumblex_movement::IGait> active_gait_ = nullptr;

    rumblex_interfaces::msg::MovementRequest active_request_;
    rumblex_interfaces::msg::MovementRequest pending_request_;

    rclcpp::Publisher<rumblex_interfaces::msg::MovementRequest>::SharedPtr movement_type_pub_;
};

}  // namespace rumblex_movement