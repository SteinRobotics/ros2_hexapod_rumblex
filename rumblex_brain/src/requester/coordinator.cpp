/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "requester/coordinator.hpp"

#include <ament_index_cpp/get_package_share_path.hpp>
#include <cmath>
#include <format>
#include <stdexcept>
#include <string>

using namespace rumblex_interfaces::msg;
using namespace std::chrono_literals;

namespace brain {

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using mp_units::angular::unit_symbols::deg;
using mp_units::angular::unit_symbols::rad;

CCoordinator::CCoordinator(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CActionPlanner> actionPlanner)
    : node_(node), actionPlanner_(actionPlanner) {
    const auto required = [&node](const char* name) {
        const double value = node->has_parameter(name) ? node->get_parameter(name).as_double()
                                                       : node->declare_parameter<double>(name);
        if (!std::isfinite(value)) {
            throw std::invalid_argument(std::string(name) + " must be finite");
        }
        return value;
    };
    kMaxVelocityLinear_ = required("max_velocity_linear") * m / s;
    kMaxVelocityRotation_ = required("max_velocity_rotation") * rad / s;
    kBodyFactorHeight_ = required("body_factor_height") * m;
    kJoystickDeadzone_ = required("joystick_deadzone");
    kMinBodyHeight_ = required("min_body_height") * m;
    kMaxBodyHeight_ = required("max_body_height") * m;

    if (kMaxVelocityLinear_ <= 0.0 * m / s)
        throw std::invalid_argument("max_velocity_linear must be positive");
    if (kMaxVelocityRotation_ <= 0.0 * rad / s)
        throw std::invalid_argument("max_velocity_rotation must be positive");
    if (kBodyFactorHeight_ < 0.0 * m) throw std::invalid_argument("body_factor_height must be nonnegative");
    if (kJoystickDeadzone_ < 0.0 || kJoystickDeadzone_ >= 1.0) {
        throw std::invalid_argument("joystick_deadzone must be in [0, 1)");
    }
    if (kMinBodyHeight_ > 0.0 * m) throw std::invalid_argument("min_body_height must be <= 0");
    if (kMaxBodyHeight_ < 0.0 * m) throw std::invalid_argument("max_body_height must be >= 0");

    textInterpreter_ = std::make_shared<CTextInterpreter>(node_);
    errorManagement_ = std::make_shared<CErrorManagement>(node_);
    sequenceParser_ = std::make_shared<CSequenceParser>(node_);

    if (node->declare_parameter<bool>("autostart_listening")) {
        auto request = std::make_shared<RequestListening>();
        request->active = true;
        submitRequest(request, Prio::Background);
    }

    kActivateMovementWaiting_ = node->declare_parameter<bool>("activate_movement_waiting");

    loadSequences();

    timerErrorRequest_ = std::make_shared<CSimpleTimer>();
    timerNoRequest_ = std::make_shared<CSimpleTimer>(kActivateMovementWaiting_);
}

void CCoordinator::loadSequences() {
    const auto package_share_path = ament_index_cpp::get_package_share_path("rumblex_brain");
    const auto file_path = (package_share_path / "config" / "sequences").string();

    if (!sequenceParser_->parseDirectory(file_path)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to parse sequences from: %s", file_path.c_str());
        return;
    }
}

void CCoordinator::executeSequence(const Sequence& sequence, Prio prio) {
    RCLCPP_INFO(node_->get_logger(), "Starting sequence from catalog: %s", sequence.name.c_str());

    if (sequence.name == "standup") {
        isStanding_ = true;
    } else if (sequence.name == "laydown") {
        isStanding_ = false;
    }

    const auto invocation = actionPlanner_->nextInvocationId();
    // Submit all action groups from the sequence
    for (const auto& actionGroup : sequence.actionGroups) {
        std::vector<std::shared_ptr<RequestBase>> requests;
        for (const auto& request : actionGroup) {
            requests.push_back(request);
        }
        actionPlanner_->request(requests, prio, invocation);
    }
}

void CCoordinator::sequenceRequested(const std::string& name) {
    const auto sequence = sequenceParser_->getSequence(name);
    if (!sequence) {
        RCLCPP_WARN(node_->get_logger(), "Unknown sequence requested: %s", name.c_str());
        return;
    }
    executeSequence(sequence->get(), name == "standup" ? Prio::High : Prio::Normal);
}

void CCoordinator::cycleGaitMode() {
    activeGaitIndex_ = (activeGaitIndex_ + 1) % gaitModes_.size();
    auto gaitName = movementTypeToName.at(gaitModes_[activeGaitIndex_]);
    RCLCPP_INFO(node_->get_logger(), "Gait mode switched to: %s", gaitName.c_str());
    auto request = std::make_shared<RequestTalking>();
    request->text = gaitName;
    std::erase_if(request->text, [](unsigned char c) {
        return !((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'));
    });
    submitRequest(request, Prio::High);
}

void CCoordinator::cmdVelReceived(const geometry_msgs::msg::Twist& msg) {
    submitRequestMove({.type = MovementRequest::CONTINUOUS_MOVE,
                       .duration = 0.0 * s,
                       .priority = Prio::High,
                       .velocity = msg});
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

    // Check if joystick request matches a sequence from YAML
    auto sequence = sequenceParser_->getSequenceForJoystickRequest(msg);
    if (sequence) {
        if (sequence->get().name == "standup") {
            executeSequence(sequence->get(), Prio::High);
        } else {
            executeSequence(sequence->get(), Prio::Normal);
        }
        return;
    }

    if (gaitModes_[activeGaitIndex_] == MovementRequest::CONTINUOUS_POSE) {
        submitRequestMove(poseFromJoystick(msg));
        return;
    }

    const auto command = movementFromJoystick(msg);
    if (movement_deadline_ && actualMovementType_ == command.type) {
        RCLCPP_WARN_STREAM(node_->get_logger(), "movement request is locked, ignoring joystick request");
        return;
    }
    if (actualMovementType_ == MovementRequest::CONTINUOUS_MOVE &&
        command.type == MovementRequest::NO_REQUEST) {
        RCLCPP_INFO_STREAM(node_->get_logger(), "end move request");
        auto request = std::make_shared<RequestVelocity>();
        request->velocity = *command.velocity;
        submitRequest(request, Prio::Normal);
        return;
    }
    if (command.type == MovementRequest::NO_REQUEST && actualMovementType_ == MovementRequest::NO_REQUEST)
        return;
    submitRequestMove(command);
}

double CCoordinator::applyDeadzone(double value) const {
    return std::abs(value) > kJoystickDeadzone_ ? value : 0.0;
}

CCoordinator::MovementCommand CCoordinator::poseFromJoystick(const JoystickRequest& msg) const {
    constexpr auto max_translation = 0.05 * m;
    constexpr auto max_rotation = 20.0 * deg;
    CPose body;
    COrientation head;
    // Torso axes: +X forward, +Y left; right stick deflection is negative yaw.
    body.position.x = applyDeadzone(msg.left_stick_vertical) * max_translation;
    body.position.y = -applyDeadzone(msg.left_stick_horizontal) * max_translation;
    head.yaw = -applyDeadzone(msg.right_stick_horizontal) * max_rotation;
    head.pitch = applyDeadzone(msg.right_stick_vertical) * max_rotation;

    MovementCommand command{.type = MovementRequest::CONTINUOUS_POSE};
    // Neutral sticks leave previously supplied pose targets unchanged.
    if (body.position.x != 0.0 * m || body.position.y != 0.0 * m) {
        command.body_pose.has_torso_pose = true;
        command.body_pose.torso_pose = body.toMsg();
    }
    if (head.yaw != 0.0 * deg || head.pitch != 0.0 * deg) {
        command.body_pose.has_head_orientation = true;
        command.body_pose.head_orientation = head.toMsg();
    }
    return command;
}

CCoordinator::MovementCommand CCoordinator::movementFromJoystick(const JoystickRequest& msg) const {
    const double forward = applyDeadzone(msg.left_stick_vertical);
    const double lateral = applyDeadzone(msg.left_stick_horizontal);
    const double yaw = applyDeadzone(msg.right_stick_horizontal);
    const double height = applyDeadzone(msg.right_stick_vertical);
    Velocity velocity;
    velocity.linear.x = forward * kMaxVelocityLinear_;
    velocity.linear.y = -lateral * kMaxVelocityLinear_;
    velocity.angular.z = -yaw * kMaxVelocityRotation_;
    CPose body;
    body.position.z = std::clamp(height * kBodyFactorHeight_, kMinBodyHeight_, kMaxBodyHeight_);
    const bool active = forward != 0.0 || lateral != 0.0 || yaw != 0.0 || height != 0.0;
    rumblex_interfaces::msg::BodyPose pose;
    pose.has_torso_pose = pose.has_head_orientation = true;
    pose.torso_pose = body.toMsg();
    return {.type = active ? gaitModes_[activeGaitIndex_] : uint32_t{MovementRequest::NO_REQUEST},
            .body_pose = pose,
            .velocity = velocity.toMsg()};
}

void CCoordinator::speechRecognized(std::string text) {
    auto identifiedWords = textInterpreter_->parseText(text);
    auto command = textInterpreter_->searchInterpretation(identifiedWords);
    RCLCPP_INFO_STREAM(node_->get_logger(), "next command is: " << command);

    // Check if command matches a sequence from YAML
    auto sequence = sequenceParser_->getSequenceForVoiceRequest(command);
    if (sequence) {
        executeSequence(sequence->get(), Prio::High);
        return;
    }

    if (command == "commandMove" || command == "commandRun") {
        constexpr auto kVelocityLinear_ = 0.005 * m / s;
        Velocity velocity;
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
        const bool running = command == "commandRun";
        submitRequestMove(
            {.type = running ? MovementRequest::CONTINUOUS_RUNNING : MovementRequest::CONTINUOUS_MOVE,
             .duration = 0 * s,
             .comment = running ? "ich renne los" : "ich laufe los",
             .priority = Prio::Normal,
             .velocity = velocity.toMsg()});
    } else if (command == "commandBite" || command == "commandStomp") {
        RCLCPP_WARN_STREAM(node_->get_logger(), "No gait implemented for voice command: " << command);
    } else if (command == "commandStopMove") {
        RCLCPP_INFO_STREAM(node_->get_logger(), "submit stop move request");
        Velocity velocity;
        submitRequestMove({.type = MovementRequest::CONTINUOUS_MOVE,
                           .duration = 0 * s,
                           .comment = "ich halte an",
                           .priority = Prio::Normal,
                           .velocity = velocity.toMsg()});

    } else if (command == "tellMeSupplyVoltage") {
        requestTellSupplyVoltage(Prio::Normal);
    } else if (command == "tellMeServoVoltage") {
        requestTellServoVoltage(Prio::Normal);
    } else if (command == "tellMeServoTemperature") {
        requestTellServoTemperature(Prio::Normal);
    } else if (command == "musicOn") {
        requestMusikOn(Prio::Normal);
    } else if (command == "musicOff") {
        requestMusikOff(Prio::High);
    }
}

void CCoordinator::supplyVoltageReceived(float voltage) {
    auto statusSupplyVoltage = errorManagement_->filterSupplyVoltage(static_cast<double>(voltage) * V);
    if (statusSupplyVoltage == EError::VoltageCriticalLow) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "SupplyVoltage is critical low, shutting down the system");

        std::string text = "Versorgungsspannung extrem niedrig, schalte ab";
        requestReactionOnError(text,
                               {.turn_off_servo_relay = true, .shutdown = true, .priority = Prio::Highest});
        return;
    }

    if (statusSupplyVoltage == EError::VoltageLow) {
        RCLCPP_WARN_STREAM(node_->get_logger(), "SupplyVoltage is low");
        std::string text = "Versorgungsspannung ist zu niedrig";
        requestReactionOnError(text, {.priority = Prio::High});
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
        requestReactionOnError(text, {.turn_off_servo_relay = true, .priority = Prio::Highest});
    } else {
        text = "Servo Fehler " + errorManagement_->getErrorName(error);
        requestReactionOnError(text, {.turn_off_servo_relay = true, .priority = Prio::High});
    }
}

void CCoordinator::movementTypeActualReceived(const MovementRequest& msg) {
    // RCLCPP_INFO_STREAM(node_->get_logger(), "movementTypeActualReceived: " << msg.name);
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

void CCoordinator::requestShutdown(Prio prio) {
    std::string text = "Ich muss mich jetzt abschalten";
    auto request = std::make_shared<RequestTalking>();
    request->text = text;
    submitRequest(request, prio);

    if (isStanding_) {
        submitRequestMove({.type = MovementRequest::SEQUENCE_LAYDOWN, .duration = 1.5 * s, .priority = prio});
    }
    auto sysRequest = std::make_shared<RequestSystem>();
    sysRequest->turnOffServoRelay = true;
    isServoRelayOn_ = false;
    sysRequest->systemShutdown = true;
    submitRequest(sysRequest, prio);
}

void CCoordinator::requestReactionOnError(std::string text, const ErrorReaction& reaction) {
    const auto prio = reaction.priority;
    if (timerErrorRequest_->isRunning() && !timerErrorRequest_->haveSecondsElapsed(10.0)) {
        return;
    } else {
        timerErrorRequest_->start();
    }

    auto talkRequest = std::make_shared<RequestTalking>();
    talkRequest->text = text;
    submitRequest(talkRequest, prio);

    if (reaction.turn_off_servo_relay || reaction.shutdown) {
        if (isStanding_) {
            submitRequestMove(
                {.type = MovementRequest::SEQUENCE_LAYDOWN, .duration = 1.5 * s, .priority = prio});
        }
        auto sysRequest = std::make_shared<RequestSystem>();
        sysRequest->turnOffServoRelay = reaction.turn_off_servo_relay;
        isServoRelayOn_ = !reaction.turn_off_servo_relay;
        sysRequest->systemShutdown = reaction.shutdown;
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

void CCoordinator::submitRequestMove(const MovementCommand& command) {
    if (!isfinite(command.duration)) {
        throw std::invalid_argument("movement duration must be finite");
    }
    std::vector<std::shared_ptr<RequestBase>> request_v;
    if (!command.comment.empty()) {
        auto talkRequest = std::make_shared<RequestTalking>();
        talkRequest->text = command.comment;
        request_v.push_back(talkRequest);
    }
    // If we are not standing, we need to stand up first
    if (!isStanding_ && (command.type == MovementRequest::CONTINUOUS_MOVE ||
                         command.type == MovementRequest::CONTINUOUS_RUNNING)) {
        RCLCPP_INFO_STREAM(node_->get_logger(), "standup before move request");
        isStanding_ = true;
        // recursive call to first stand up
        submitRequestMove({.type = MovementRequest::SEQUENCE_STAND_UP,
                           .duration = 1.5 * s,
                           .comment = "ich stehe erst mal auf",
                           .priority = command.priority});
    }

    auto request = MovementRequest();
    request.type = command.type;
    request.duration_s = command.duration;
    if (command.direction.has_value()) {
        request.direction = command.direction.value();
    }
    request.name = movementTypeToName.at(request.type);
    request.body_pose = command.body_pose;
    auto moveRequest = std::make_shared<RequestMovementType>();
    moveRequest->movementRequest = request;
    request_v.push_back(moveRequest);
    if (command.velocity.has_value()) {
        auto velocityRequest = std::make_shared<RequestVelocity>();
        velocityRequest->velocity = command.velocity.value();
        request_v.push_back(velocityRequest);
    }
    actionPlanner_->request(request_v, command.priority);

    if (command.type == MovementRequest::CONTINUOUS_MOVE ||
        command.type == MovementRequest::CONTINUOUS_RUNNING ||
        (command.type == MovementRequest::CONTINUOUS_POSE && command.duration <= 0.0 * s)) {
        movement_deadline_.reset();
        return;
    }
    movement_deadline_ = MovementDeadline{
        std::chrono::steady_clock::now() +
            std::chrono::duration<double>(std::max(0.0 * s, command.duration).numerical_value_in(s)),
        command.type};
}

void CCoordinator::requestTellSupplyVoltage(Prio prio) {
    auto text = std::format("Die Versorgungsspannung ist aktuell {:.1f} Volt",
                            errorManagement_->getFilteredSupplyVoltage().numerical_value_in(V));
    auto request = std::make_shared<RequestTalking>();
    request->text = text;
    submitRequest(request, prio);
}

void CCoordinator::requestTellServoVoltage(Prio prio) {
    auto text = std::format("Die Servo Spannung ist aktuell {:.1f} Volt",
                            errorManagement_->getFilteredServoVoltage().numerical_value_in(V));
    auto request = std::make_shared<RequestTalking>();
    request->text = text;
    submitRequest(request, prio);
}

void CCoordinator::requestTellServoTemperature(Prio prio) {
    auto text = std::format("Die Servo Temperatur ist aktuell {:.0f} Grad",
                            units::inCelsius(errorManagement_->getFilteredServoTemperature()));
    auto request = std::make_shared<RequestTalking>();
    request->text = text;
    submitRequest(request, prio);
}

// ----------------------------------------------------------------------------
//  update
// ---------------------------------------------------------------------------
void CCoordinator::update() {
    if (movement_deadline_ && std::chrono::steady_clock::now() >= movement_deadline_->time) {
        if (actualMovementType_ == movement_deadline_->movement_type) {
            actualMovementType_ = MovementRequest::NO_REQUEST;
        }
        movement_deadline_.reset();
    }
    if (kActivateMovementWaiting_ && actualMovementType_ == MovementRequest::NO_REQUEST) {
        if (!timerNoRequest_->isRunning()) {
            timerNoRequest_->start();
        }

        // If no request is received for 30 seconds, we request a default move
        if (timerNoRequest_->haveSecondsElapsed(30.0)) {
            if (movement_deadline_) {
                RCLCPP_WARN_STREAM(node_->get_logger(),
                                   "No request received for 30 seconds, but movement request is locked");
                return;
            }
            RCLCPP_INFO_STREAM(node_->get_logger(), "No request received for 30 seconds, requesting default");
        }

    } else {
        timerNoRequest_->stop();
    }
}

}  // namespace brain
