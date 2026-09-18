/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "requester/coordinator.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <format>

using namespace rumblex_interfaces::msg;
using namespace std::chrono_literals;

namespace brain {

CCoordinator::CCoordinator(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CActionPlanner> actionPlanner)
    : node_(node), actionPlanner_(actionPlanner) {
    textInterpreter_ = std::make_shared<CTextInterpreter>(node_);
    errorManagement_ = std::make_shared<CErrorManagement>(node_);
    behaviorParser_ = std::make_shared<CBehaviorParser>(node_);

    kMaxVelocityLinear_ = node->declare_parameter<double>("max_velocity_linear", rclcpp::PARAMETER_DOUBLE);
    kMaxVelocityRotation_ =
        node->declare_parameter<double>("max_velocity_rotation", rclcpp::PARAMETER_DOUBLE);
    kBodyFactorHeight_ = node->declare_parameter<double>("body_factor_height", rclcpp::PARAMETER_DOUBLE);
    kJoystickDeadzone_ = node->declare_parameter<double>("joystick_deadzone", rclcpp::PARAMETER_DOUBLE);
    kMinBodyHeight_ =
        node->declare_parameter<double>("min_body_height", rclcpp::PARAMETER_DOUBLE);  // negative value
    kMaxBodyHeight_ =
        node->declare_parameter<double>("max_body_height", rclcpp::PARAMETER_DOUBLE);  // positive value

    if (node->declare_parameter<bool>("autostart_listening")) {
        auto request = std::make_shared<RequestListening>();
        request->active = true;
        submitRequest(request, Prio::Background);
    }

    kActivateMovementWaiting_ = node->declare_parameter<bool>("activate_movement_waiting");

    loadBehaviors();

    timerMovementRequest_ = std::make_shared<CCallbackTimer>();
    timerErrorRequest_ = std::make_shared<CSimpleTimer>();
    timerNoRequest_ = std::make_shared<CSimpleTimer>(kActivateMovementWaiting_);
}

void CCoordinator::loadBehaviors() {
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("rumblex_brain");
    std::string file_path = package_share_directory + "/config/behaviors.json";

    if (!behaviorParser_->parseFile(file_path)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to parse behaviors from: %s", file_path.c_str());
        return;
    }
}

void CCoordinator::executeBehavior(const Behavior& behavior, Prio prio) {
    RCLCPP_INFO(node_->get_logger(), "Executing behavior: %s", behavior.name.c_str());

    if (behavior.name == "standup") {
        isStanding_ = true;
    } else if (behavior.name == "laydown") {
        isStanding_ = false;
    }

    // Submit all action groups from the behavior
    for (const auto& actionGroup : behavior.actionGroups) {
        std::vector<std::shared_ptr<RequestBase>> requests;
        for (const auto& request : actionGroup) {
            requests.push_back(request);
        }
        actionPlanner_->request(requests, prio);
    }
}

void CCoordinator::cycleGaitMode() {
    activeGaitIndex_ = (activeGaitIndex_ + 1) % gaitModes_.size();
    auto gaitName = movementTypeToName.at(gaitModes_[activeGaitIndex_]);
    RCLCPP_INFO(node_->get_logger(), "Gait mode switched to: %s", gaitName.c_str());
    auto request = std::make_shared<RequestTalking>();
    request->text = gaitName;
    submitRequest(request, Prio::High);
}

void CCoordinator::cmdVelReceived(const geometry_msgs::msg::Twist& msg) {
    submitRequestMove(MovementRequest::CONTINUOUS_MOVE, 0.0, "", Prio::High, std::nullopt, std::nullopt, msg);
}

void CCoordinator::joystickRequestReceived(const JoystickRequest& msg) {
    if (msg.button_long_select) {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Shutdown requested by joystick");
        requestShutdown(Prio::High);
        return;
    }

    // Cycle gait mode on button_start
    if (msg.button_start) {
        cycleGaitMode();
        return;
    }

    // Check if joystick request matches a behavior from JSON
    auto behavior = behaviorParser_->getBehaviorForJoystickRequest(msg);
    if (behavior) {
        executeBehavior(behavior->get(), Prio::High);
        return;
    }

    double duration_s = 0.0;
    std::string comment = "";
    uint32_t newMovementType = MovementRequest::NO_REQUEST;
    auto activeGait = gaitModes_[activeGaitIndex_];
    auto body = rumblex_interfaces::msg::Pose();
    auto head = rumblex_interfaces::msg::Orientation();
    auto velocity = geometry_msgs::msg::Twist();
    std::optional<uint8_t> direction = std::nullopt;

    // MOVE mode: the combined gait handles velocity-based sub-gait selection internally

    if (gaitModes_[activeGaitIndex_] == MovementRequest::CONTINUOUS_POSE) {
        // LEFT_STICK -> linear movement
        // float32 left_stick_vertical   # TOP  = -1.0, DOWN = 1.0,  hangs on 0.004 -> means 0.0
        const auto max_displacement_m = 0.05;  // meters
        if (std::abs(msg.left_stick_vertical) > kJoystickDeadzone_) {
            body.position.x = msg.left_stick_vertical * max_displacement_m;
        }
        // float32 left_stick_horizontal # LEFT = -1.0, RIGHT = 1.0, hangs on 0.004 -> means 0.0
        if (std::abs(msg.left_stick_horizontal) > kJoystickDeadzone_) {
            body.position.y = msg.left_stick_horizontal * max_displacement_m;
        }
        // RIGHT_STICK -> rotation
        // float32 right_stick_horizontal  # LEFT = -1.0, RIGHT = 1.0, hangs on 0.004 -> means 0.0
        const auto max_displacement_deg = 20.0;  // degrees
        if (std::abs(msg.right_stick_horizontal) > kJoystickDeadzone_) {
            head.yaw = msg.right_stick_horizontal * max_displacement_deg;
        }
        // float32 right_stick_vertical    # TOP  = -1.0, DOWN = 1.0, hangs on 0.004 -> means 0.0
        if (std::abs(msg.right_stick_vertical) > kJoystickDeadzone_) {
            head.pitch = msg.right_stick_vertical * max_displacement_deg;
        }
        if (body.position.x != 0.0 || body.position.y != 0.0) {
            auto request_body = std::make_shared<RequestSinglePose>();
            request_body->pose = body;
            submitRequest(request_body, Prio::High);
        }
        if (head.yaw != 0.0 || head.pitch != 0.0) {
            auto request_head = std::make_shared<RequestHeadOrientation>();
            request_head->orientation = head;
            submitRequest(request_head, Prio::High);
        }
        return;
    }

    // LEFT_STICK -> linear movement
    // float32 left_stick_vertical   # TOP  = -1.0, DOWN = 1.0,  hangs on 0.004 -> means 0.0
    if (std::abs(msg.left_stick_vertical) > kJoystickDeadzone_) {
        velocity.linear.x = msg.left_stick_vertical * kMaxVelocityLinear_;
        newMovementType = activeGait;
    }
    // float32 left_stick_horizontal # LEFT = -1.0, RIGHT = 1.0, hangs on 0.004 -> means 0.0
    if (std::abs(msg.left_stick_horizontal) > kJoystickDeadzone_) {
        velocity.linear.y = msg.left_stick_horizontal * kMaxVelocityLinear_;
        newMovementType = activeGait;
    }
    // RIGHT_STICK -> rotation
    // float32 right_stick_horizontal  # LEFT = -1.0, RIGHT = 1.0, hangs on 0.004 -> means 0.0
    if (std::abs(msg.right_stick_horizontal) > kJoystickDeadzone_) {
        velocity.angular.z = msg.right_stick_horizontal * kMaxVelocityRotation_;
        newMovementType = activeGait;
    }

    // float32 right_stick_vertical    # TOP  = -1.0, DOWN = 1.0, hangs on 0.004 -> means 0.0
    if (std::abs(msg.right_stick_vertical) > kJoystickDeadzone_) {
        body.position.z =
            std::clamp(msg.right_stick_vertical * kBodyFactorHeight_, kMinBodyHeight_, kMaxBodyHeight_);
        newMovementType = activeGait;
    } else {
        body.position.z = 0.0;
    }

    if (isNewMoveRequestLocked_ && actualMovementType_ == newMovementType) {
        RCLCPP_WARN_STREAM(node_->get_logger(), "isNewMoveRequestLocked is true, ignoring joystick request");
        return;
    }

    if ((actualMovementType_ == MovementRequest::CONTINUOUS_MOVE) && newMovementType == MovementRequest::NO_REQUEST) {
        RCLCPP_INFO_STREAM(node_->get_logger(), "end move request");
        auto request = std::make_shared<RequestVelocity>();
        request->velocity = velocity;
        submitRequest(request, Prio::High);
        return;
    }

    // no new request
    if (newMovementType == MovementRequest::NO_REQUEST &&
        actualMovementType_ == MovementRequest::NO_REQUEST) {
        return;
    }
    submitRequestMove(newMovementType, duration_s, comment, Prio::High, body, head, velocity, direction);
}

void CCoordinator::speechRecognized(std::string text) {
    auto identifiedWords = textInterpreter_->parseText(text);
    auto command = textInterpreter_->searchInterpretation(identifiedWords);
    RCLCPP_INFO_STREAM(node_->get_logger(), "next command is: " << command);

    // Check if command matches a behavior from JSON
    auto behavior = behaviorParser_->getBehaviorForVoiceRequest(command);
    if (behavior) {
        executeBehavior(behavior->get(), Prio::High);
        return;
    }

    if (command == "commandMove") {
        constexpr double kVelocityLinear_ = 0.005;
        geometry_msgs::msg::Twist velocity;
        if (textInterpreter_->lettersIdentified("vorn", identifiedWords) ||
            textInterpreter_->lettersIdentified("vorne", identifiedWords)) {
            velocity.linear.x = kVelocityLinear_;
        } else if (textInterpreter_->lettersIdentified("hinten", identifiedWords)) {
            velocity.linear.x = -kVelocityLinear_;
        }
        if (textInterpreter_->lettersIdentified("links", identifiedWords)) {
            velocity.linear.y = kVelocityLinear_;
        } else if (textInterpreter_->lettersIdentified("rechts", identifiedWords)) {
            velocity.linear.y = -kVelocityLinear_;
        }
        RCLCPP_INFO_STREAM(node_->get_logger(), "submit move request");
        submitRequestMove(MovementRequest::CONTINUOUS_MOVE, 0, "ich laufe los", Prio::High, std::nullopt, std::nullopt,
                          velocity);
    } else if (command == "commandStopMove") {
        RCLCPP_INFO_STREAM(node_->get_logger(), "submit stop move request");
        geometry_msgs::msg::Twist velocity;
        submitRequestMove(MovementRequest::CONTINUOUS_MOVE, 0, "ich halte an", Prio::High, std::nullopt, std::nullopt,
                          velocity);

    } else if (command == "tellMeSupplyVoltage") {
        requestTellSupplyVoltage(Prio::High);
    } else if (command == "tellMeServoVoltage") {
        requestTellServoVoltage(Prio::High);
    } else if (command == "tellMeServoTemperature") {
        requestTellServoTemperature(Prio::High);
    } else if (command == "musicOn") {
        requestMusikOn();
    } else if (command == "musicOff") {
        requestMusikOff();
    }
}

void CCoordinator::supplyVoltageReceived(float voltage) {
    auto statusSupplyVoltage = errorManagement_->filterSupplyVoltage(voltage);
    if (statusSupplyVoltage == EError::VoltageCriticalLow) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "SupplyVoltage is critical low, shutting down the system");

        std::string text = "Versorgungsspannung extrem niedrig, schalte ab";
        requestReactionOnError(text, true, true, Prio::Highest);
        return;
    }

    if (statusSupplyVoltage == EError::VoltageLow) {
        RCLCPP_WARN_STREAM(node_->get_logger(), "SupplyVoltage is low");
        std::string text = "Versorgungsspannung ist zu niedrig";
        requestReactionOnError(text, false, false, Prio::High);
    }
}

void CCoordinator::servoStatusReceived(const ServoStatus& msg) {
    if (!isServoRelayOn_) {
        return;
    }
    std::string text = "";
    auto error = errorManagement_->getErrorServo(msg);
    if (error == EError::None) {
        return;
    }
    if (error == EError::VoltageCriticalLow) {
        text = "Servo Spannung ist zu niedrig";
        requestReactionOnError(text, true, false, Prio::Highest);
    } else {
        text = "Servo Fehler " + errorManagement_->getErrorName(error);
        requestReactionOnError(text, true, false, Prio::High);
    }
}

void CCoordinator::movementTypeActualReceived(const MovementRequest& msg) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "movementTypeActualReceived: " << msg.name);
    actualMovementType_ = msg.type;
    if (actualMovementType_ == MovementRequest::SEQUENCE_STAND_UP) {
        isStanding_ = true;
    } else if (actualMovementType_ == MovementRequest::SEQUENCE_LAYDOWN) {
        isStanding_ = false;
    }
}

// ----------------------------------------------------------------------------
//  Requests
// ---------------------------------------------------------------------------
// Helper to submit a single request
void CCoordinator::submitRequest(std::shared_ptr<RequestBase> request, Prio prio) {
    std::vector<std::shared_ptr<RequestBase>> request_v;
    request_v.push_back(request);
    actionPlanner_->request(request_v, prio);
}

void CCoordinator::requestNotFound(std::string textRecognized, Prio prio) {
    std::string textOutput = "Ich habe folgendes nicht verstanden " + textRecognized;
    auto request = std::make_shared<RequestTalking>();
    request->text = textOutput;
    submitRequest(request, prio);
}

void CCoordinator::requestShutdown(Prio prio) {
    std::string text = "Ich muss mich jetzt abschalten";
    auto request = std::make_shared<RequestTalking>();
    request->text = text;
    submitRequest(request, prio);

    if (isStanding_) {
        submitRequestMove(MovementRequest::SEQUENCE_LAYDOWN, 1.5, "", prio);
    }
    auto sysRequest = std::make_shared<RequestSystem>();
    sysRequest->turnOffServoRelay = true;
    isServoRelayOn_ = false;
    sysRequest->systemShutdown = true;
    submitRequest(sysRequest, prio);
}

void CCoordinator::requestReactionOnError(std::string text, bool switchServoRelayOff,
                                          bool isShutdownRequested, Prio prio) {
    if (timerErrorRequest_->isRunning() && !timerErrorRequest_->haveSecondsElapsed(10.0)) {
        return;
    } else {
        timerErrorRequest_->start();
    }

    auto talkRequest = std::make_shared<RequestTalking>();
    talkRequest->text = text;
    submitRequest(talkRequest, prio);

    if (switchServoRelayOff || isShutdownRequested) {
        if (isStanding_) {
            submitRequestMove(MovementRequest::SEQUENCE_LAYDOWN, 1.5, "", prio);
        }
        auto sysRequest = std::make_shared<RequestSystem>();
        sysRequest->turnOffServoRelay = switchServoRelayOff;
        isServoRelayOn_ = !switchServoRelayOff;
        sysRequest->systemShutdown = isShutdownRequested;
        submitRequest(sysRequest, prio);
    }
}

void CCoordinator::requestMusikOn(Prio prio) {
    std::string song = "musicfox_hot_dogs_for_breakfast.mp3";
    auto request = std::make_shared<RequestMusic>();
    request->song = song;
    actionPlanner_->request({request}, prio);
}

void CCoordinator::requestMusikOff(Prio prio) {
    auto request = std::make_shared<RequestMusic>();
    request->song = "STOP";
    actionPlanner_->request({request}, prio);
}

void CCoordinator::requestTalking(std::string text, Prio prio) {
    auto request = std::make_shared<RequestTalking>();
    request->text = text;
    actionPlanner_->request({request}, prio);
}

void CCoordinator::requestChat(std::string text, Prio prio) {
    auto request = std::make_shared<RequestChat>();
    request->text = text;
    actionPlanner_->request({request}, prio);
}

void CCoordinator::requestWaiting(Prio prio) {
    actualMovementType_ = MovementRequest::SEQUENCE_WAITING;
    submitRequestMove(actualMovementType_, 5.0, "ich warte", prio);
}

void CCoordinator::submitRequestMove(uint32_t movementType, double duration_s, std::string comment, Prio prio,
                                     std::optional<rumblex_interfaces::msg::Pose> body,
                                     std::optional<rumblex_interfaces::msg::Orientation> head,
                                     std::optional<geometry_msgs::msg::Twist> velocity,
                                     std::optional<uint8_t> direction) {
    std::vector<std::shared_ptr<RequestBase>> request_v;
    if (!comment.empty()) {
        auto talkRequest = std::make_shared<RequestTalking>();
        talkRequest->text = comment;
        request_v.push_back(talkRequest);
    }
    // If we are not standing, we need to stand up first
    if (!isStanding_ && movementType == MovementRequest::CONTINUOUS_MOVE) {
        RCLCPP_INFO_STREAM(node_->get_logger(), "standup before move request");
        isStanding_ = true;
        // recursive call to first stand up
        submitRequestMove(MovementRequest::SEQUENCE_STAND_UP, 1.5, "ich stehe erst mal auf", prio);
    }

    auto request = MovementRequest();
    request.type = movementType;
    request.duration_s = duration_s;
    if (direction.has_value()) {
        request.direction = direction.value();
    }
    request.name = movementTypeToName.at(request.type);
    auto moveRequest = std::make_shared<RequestMovementType>();
    moveRequest->movementRequest = request;
    request_v.push_back(moveRequest);
    if (body.has_value()) {
        auto bodyRequest = std::make_shared<RequestSinglePose>();
        bodyRequest->pose = body.value();
        request_v.push_back(bodyRequest);
    }
    if (head.has_value()) {
        auto headRequest = std::make_shared<RequestHeadOrientation>();
        headRequest->orientation = head.value();
        request_v.push_back(headRequest);
    }
    if (velocity.has_value()) {
        auto velocityRequest = std::make_shared<RequestVelocity>();
        velocityRequest->velocity = velocity.value();
        request_v.push_back(velocityRequest);
    }
    actionPlanner_->request(request_v, prio);

    // Lock the new move request for the given duration except for move gait requests
    if (MovementRequest::CONTINUOUS_MOVE == movementType) {
        return;
    }
    isNewMoveRequestLocked_ = true;
    // RCLCPP_INFO_STREAM(node_->get_logger(), "isNewMoveRequestLocked_ locked");
    timerMovementRequest_->waitSecondsNonBlocking(duration_s, [this]() {
        isNewMoveRequestLocked_ = false;
        actualMovementType_ = MovementRequest::NO_REQUEST;
        // RCLCPP_INFO_STREAM(node_->get_logger(), "isNewMoveRequestLocked_ released");
    });
}

void CCoordinator::requestTellSupplyVoltage(Prio prio) {
    auto text = std::format("Die Versorgungsspannung ist aktuell {:.1f} Volt",
                            errorManagement_->getFilteredSupplyVoltage());
    auto request = std::make_shared<RequestTalking>();
    request->text = text;
    submitRequest(request, prio);
}

void CCoordinator::requestTellServoVoltage(Prio prio) {
    auto text = std::format("Die Servo Spannung ist aktuell {:.1f} Volt",
                            errorManagement_->getFilteredServoVoltage());
    auto request = std::make_shared<RequestTalking>();
    request->text = text;
    submitRequest(request, prio);
}

void CCoordinator::requestTellServoTemperature(Prio prio) {
    auto text = std::format("Die Servo Temperatur ist aktuell {:.0f} Grad",
                            errorManagement_->getFilteredServoTemperature());
    auto request = std::make_shared<RequestTalking>();
    request->text = text;
    submitRequest(request, prio);
}

// ----------------------------------------------------------------------------
//  update
// ---------------------------------------------------------------------------
void CCoordinator::update() {
    if (kActivateMovementWaiting_ && actualMovementType_ == MovementRequest::NO_REQUEST) {
        if (!timerNoRequest_->isRunning()) {
            timerNoRequest_->start();
        }

        // If no request is received for 30 seconds, we request a default move
        if (timerNoRequest_->haveSecondsElapsed(30.0)) {
            if (isNewMoveRequestLocked_) {
                RCLCPP_WARN_STREAM(node_->get_logger(),
                                   "No request received for 30 seconds, but isNewMoveRequestLocked_ is true");
                return;
            }
            RCLCPP_INFO_STREAM(node_->get_logger(), "No request received for 30 seconds, requesting default");
            // TODO: change all other Prios to High
            // requestWaiting(Prio::Normal);
        }

    } else {
        timerNoRequest_->stop();
    }
}

}  // namespace brain
