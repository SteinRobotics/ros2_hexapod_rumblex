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
#include "rumblex_interfaces/msg/movement_request.hpp"
#include "rumblex_utils/geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "requester/gait_bodyroll.hpp"
#include "requester/gait_clap.hpp"
#include "requester/gait_continuouspose.hpp"
#include "requester/gait_highfive.hpp"
#include "requester/gait_laydown.hpp"
#include "requester/gait_legwave.hpp"
#include "requester/gait_look.hpp"
#include "requester/gait_move_combined.hpp"
#include "requester/gait_parameters.hpp"
#include "requester/gait_running.hpp"
#include "requester/gait_singlepose.hpp"
#include "requester/gait_standup.hpp"
#include "requester/gait_testlegs.hpp"
#include "requester/gait_waiting.hpp"
#include "requester/gait_watch.hpp"
#include "requester/igaits.hpp"
#include "requester/kinematics.hpp"
#include "requester/types.hpp"

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
                            CPose body = CPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
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