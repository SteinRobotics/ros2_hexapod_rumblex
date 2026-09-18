/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include "rclcpp/rclcpp.hpp"
//
#include "rumblex_interfaces/msg/continuous_movement_update.hpp"
#include "rumblex_interfaces/msg/movement_request.hpp"
//
#include <rumblex_utils/callback_timer.hpp>

#include "ihandler.hpp"
#include "requester/irequester.hpp"

namespace brain {

class CMovement : public IHandler {
   public:
    CMovement(std::shared_ptr<rclcpp::Node> node);
    virtual ~CMovement() = default;

    void update() override;
    void cancel() override;

    void run(std::shared_ptr<RequestMovementType> request);
    void run(std::shared_ptr<RequestSinglePose> request);
    void run(std::shared_ptr<RequestHeadOrientation> request);
    void run(std::shared_ptr<RequestVelocity> request);

   private:
    void timerCallback();
    void publishMovementRequest();
    void publishContinuousUpdate();

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<rumblex_interfaces::msg::MovementRequest>::SharedPtr pub_cmd_movement_;
    rclcpp::Publisher<rumblex_interfaces::msg::ContinuousMovementUpdate>::SharedPtr pub_cmd_movement_update_;
    rumblex_interfaces::msg::MovementRequest current_request_;
    rumblex_interfaces::msg::ContinuousMovementUpdate current_continuous_update_;

    std::shared_ptr<CCallbackTimer> callback_timer_;
};

}  // namespace brain
