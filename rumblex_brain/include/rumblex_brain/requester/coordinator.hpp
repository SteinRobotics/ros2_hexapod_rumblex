/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
//
#include "rumblex_interfaces/msg/joystick_request.hpp"
#include "rumblex_interfaces/msg/movement_request.hpp"
#include "rumblex_interfaces/msg/servo_status.hpp"
//
#include <rumblex_utils/callback_timer.hpp>
#include <rumblex_utils/simpletimer.hpp>

#include "action/action_planner.hpp"
#include "irequester.hpp"
#include "parser/behavior_parser.hpp"
#include "requester/error_management.hpp"
#include "requester/text_interpreter.hpp"
#include "requester/utility.hpp"

namespace brain {

class CCoordinator : public IRequester {
   public:
    CCoordinator(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CActionPlanner> jobHandler);
    virtual ~CCoordinator() = default;

    void update() override;

    void joystickRequestReceived(const rumblex_interfaces::msg::JoystickRequest& msg);
    void cmdVelReceived(const geometry_msgs::msg::Twist& msg);
    void speechRecognized(std::string text);
    void supplyVoltageReceived(float voltage);
    void servoStatusReceived(const rumblex_interfaces::msg::ServoStatus& msg);
    void movementTypeActualReceived(const rumblex_interfaces::msg::MovementRequest& msg);

   private:
    void loadBehaviors();
    void executeBehavior(const Behavior& behavior, Prio prio = Prio::High);
    void submitRequest(std::shared_ptr<RequestBase> request, Prio prio);

    void submitRequestMove(uint32_t movementType, double duration_s = 0.0, std::string comment = "",
                           Prio prio = Prio::Normal,
                           std::optional<rumblex_interfaces::msg::Pose> body = std::nullopt,
                           std::optional<rumblex_interfaces::msg::Orientation> head = std::nullopt,
                           std::optional<geometry_msgs::msg::Twist> velocity = std::nullopt,
                           std::optional<uint8_t> direction = std::nullopt);

    void requestShutdown(Prio prio);
    void requestReactionOnError(std::string text, bool switchServoRelayOff, bool isShutdownRequested,
                                Prio prio = Prio::Normal);
    void requestNotFound(std::string textRecognized, Prio prio = Prio::Normal);

    void requestTellSupplyVoltage(Prio prio = Prio::Normal);
    void requestTellServoVoltage(Prio prio = Prio::Normal);
    void requestTellServoTemperature(Prio prio = Prio::Normal);
    void requestMusikOn(Prio prio = Prio::Normal);
    void requestMusikOff(Prio prio = Prio::Normal);
    void requestTalking(std::string text, Prio prio = Prio::Normal);
    void requestChat(std::string text, Prio prio = Prio::Normal);
    void requestWaiting(Prio prio = Prio::Normal);
    void cycleGaitMode();

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CActionPlanner> actionPlanner_;
    std::shared_ptr<CErrorManagement> errorManagement_;
    std::shared_ptr<CTextInterpreter> textInterpreter_;
    std::shared_ptr<CBehaviorParser> behaviorParser_;
    std::shared_ptr<CSimpleTimer> timerErrorRequest_;
    std::shared_ptr<CSimpleTimer> timerNoRequest_;
    std::shared_ptr<CCallbackTimer> timerMovementRequest_;

    std::atomic<bool> isNewMoveRequestLocked_{false};

    uint32_t actualMovementType_ = rumblex_interfaces::msg::MovementRequest::NO_REQUEST;
    bool isStanding_ = false;
    bool isServoRelayOn_ = true;

    // Gait cycling: button_start iterates through these modes
    const std::vector<uint32_t> gaitModes_ = {
        rumblex_interfaces::msg::MovementRequest::CONTINUOUS_MOVE,
        rumblex_interfaces::msg::MovementRequest::CONTINUOUS_POSE,
        rumblex_interfaces::msg::MovementRequest::CONTINUOUS_RUNNING,
    };
    size_t activeGaitIndex_ = 0;

    double kMaxVelocityLinear_ = 0.0;
    double kMaxVelocityRotation_ = 0.0;
    double kBodyFactorHeight_ = 0.0;
    double kMinBodyHeight_ = 0.0;
    double kMaxBodyHeight_ = 0.0;
    double kJoystickDeadzone_ = 0.0;
    bool kActivateMovementWaiting_ = false;
};

}  // namespace brain