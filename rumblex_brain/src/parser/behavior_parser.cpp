/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "rumblex_brain/parser/behavior_parser.hpp"

#include <yaml-cpp/yaml.h>

namespace brain {
namespace {
// Defaults apply only to omitted fields; malformed explicit values remain errors.
template <typename T>
T valueOr(const YAML::Node& node, const char* key, const T& fallback) {
    return node[key] ? node[key].as<T>() : fallback;
}
}  // namespace

CBehaviorParser::CBehaviorParser(rclcpp::Node::SharedPtr node) : node_(node) {
}

bool CBehaviorParser::parseFile(const std::string& filePath) {
    try {
        const auto document = YAML::LoadFile(filePath);
        if (!document.IsMap() || !document["behaviors"].IsSequence()) {
            RCLCPP_ERROR(node_->get_logger(), "Invalid YAML format: expected 'behaviors' sequence");
            return false;
        }
        return parseBehaviors(document["behaviors"]);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Exception while parsing file: %s", e.what());
        return false;
    }
}

bool CBehaviorParser::parseString(const std::string& yamlString) {
    try {
        const auto document = YAML::Load(yamlString);

        if (document.IsMap() && document["behaviors"].IsSequence()) {
            return parseBehaviors(document["behaviors"]);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Invalid YAML format: expected 'behaviors' sequence");
            return false;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Exception while parsing YAML string: %s", e.what());
        return false;
    }
}

bool CBehaviorParser::parseBehaviors(const YAML::Node& behaviorsArray) {
    if (!behaviorsArray.IsSequence()) {
        RCLCPP_ERROR(node_->get_logger(), "Expected behaviors to be an array");
        return false;
    }

    behaviors_.clear();

    for (const auto& behaviorNode : behaviorsArray) {
        if (!parseSingleBehavior(behaviorNode)) {
            RCLCPP_WARN(node_->get_logger(), "Failed to parse one behavior, continuing with others");
        }
    }

    return !behaviors_.empty();
}

bool CBehaviorParser::parseSingleBehavior(const YAML::Node& j) {
    Behavior behavior;

    if (!j.IsMap()) {
        RCLCPP_WARN(node_->get_logger(), "Expected behavior to be a mapping");
        return false;
    }

    // Parse behavior name
    if (j["name"] && j["name"].IsScalar()) {
        behavior.name = j["name"].as<std::string>();
    } else {
        RCLCPP_WARN(node_->get_logger(), "Behavior name not found or invalid");
        behavior.name = "UNKNOWN";
    }

    // Parse triggers
    BehaviorTrigger trigger;
    if (j["trigger"] && j["trigger"].IsMap()) {
        const auto& triggerNode = j["trigger"];
        if (triggerNode["joystick"] && triggerNode["joystick"].IsScalar()) {
            trigger.joystick = triggerNode["joystick"].as<std::string>();
        }
        if (triggerNode["voice"] && triggerNode["voice"].IsScalar()) {
            trigger.voice = triggerNode["voice"].as<std::string>();
        }
        behaviorTriggers_[behavior.name] = trigger;
    }

    // Parse actions
    if (j["actions"] && j["actions"].IsSequence()) {
        const auto& actionsNode = j["actions"];

        for (const auto& actionNode : actionsNode) {
            auto requests = parseActionGroup(actionNode);
            if (!requests.empty()) {
                behavior.actionGroups.push_back(requests);
            }
        }

        if (behavior.actionGroups.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "No valid action groups found for behavior: %s",
                         behavior.name.c_str());
            return false;
        }
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Actions array not found for behavior: %s", behavior.name.c_str());
        return false;
    }

    behaviors_.push_back(behavior);
    RCLCPP_INFO(node_->get_logger(), "Parsed behavior '%s' with %zu action groups", behavior.name.c_str(),
                behavior.actionGroups.size());

    return true;
}

const std::vector<Behavior>& CBehaviorParser::getBehaviors() const {
    return behaviors_;
}

std::optional<std::reference_wrapper<const Behavior>> CBehaviorParser::getBehavior(
    const std::string& name) const {
    for (const auto& behavior : behaviors_) {
        if (behavior.name == name) {
            return std::cref(behavior);
        }
    }
    return std::nullopt;
}

std::optional<std::reference_wrapper<const Behavior>> CBehaviorParser::getBehaviorForJoystickRequest(
    const rumblex_interfaces::msg::JoystickRequest& msg) const {
    // Check each behavior's joystick trigger
    for (const auto& behavior : behaviors_) {
        const auto& trigger = behaviorTriggers_.find(behavior.name);
        if (trigger == behaviorTriggers_.end()) {
            continue;
        }

        const std::string& joystickTrigger = trigger->second.joystick;

        // Match button triggers
        if (joystickTrigger == "button_a" && msg.button_a) return std::cref(behavior);
        if (joystickTrigger == "button_b" && msg.button_b) return std::cref(behavior);
        if (joystickTrigger == "button_x" && msg.button_x) return std::cref(behavior);
        if (joystickTrigger == "button_y" && msg.button_y) return std::cref(behavior);
        if (joystickTrigger == "button_l1" && msg.button_l1) return std::cref(behavior);
        if (joystickTrigger == "button_l2" && msg.button_l2) return std::cref(behavior);
        if (joystickTrigger == "button_r1" && msg.button_r1) return std::cref(behavior);
        if (joystickTrigger == "button_r2" && msg.button_r2) return std::cref(behavior);
        if (joystickTrigger == "button_select" && msg.button_select) return std::cref(behavior);
        if (joystickTrigger == "button_start" && msg.button_start) return std::cref(behavior);
        if (joystickTrigger == "button_home" && msg.button_home) return std::cref(behavior);

        // Match dpad triggers
        if (joystickTrigger == "dpad_vertical_up" && msg.dpad_vertical == 1) return std::cref(behavior);
        if (joystickTrigger == "dpad_vertical_down" && msg.dpad_vertical == -1) return std::cref(behavior);
        if (joystickTrigger == "dpad_horizontal_left" && msg.dpad_horizontal == -1)
            return std::cref(behavior);
        if (joystickTrigger == "dpad_horizontal_right" && msg.dpad_horizontal == 1)
            return std::cref(behavior);
    }

    return std::nullopt;
}

std::optional<std::reference_wrapper<const Behavior>> CBehaviorParser::getBehaviorForVoiceRequest(
    const std::string& voiceCommand) const {
    // Check each behavior's voice trigger
    for (const auto& behavior : behaviors_) {
        const auto& trigger = behaviorTriggers_.find(behavior.name);
        if (trigger == behaviorTriggers_.end()) {
            continue;
        }

        const std::string& voiceTrigger = trigger->second.voice;
        if (voiceTrigger == voiceCommand) {
            return std::cref(behavior);
        }
    }

    return std::nullopt;
}

std::vector<std::shared_ptr<RequestBase>> CBehaviorParser::parseActionGroup(const YAML::Node& actionObject) {
    std::vector<std::shared_ptr<RequestBase>> requests;

    if (!actionObject.IsMap()) {
        RCLCPP_WARN(node_->get_logger(), "Expected action group to be a mapping");
        return requests;
    }
    // Preserve the lexicographic dispatch order of the previous JSON parser.
    std::map<std::string, YAML::Node> orderedRequests;
    for (const auto& entry : actionObject) {
        orderedRequests.emplace(entry.first.as<std::string>(), entry.second);
    }
    for (const auto& [requestType, requestValue] : orderedRequests) {
        std::shared_ptr<RequestBase> request = nullptr;

        if (requestType == "RequestTalking") {
            request = createRequestTalking(requestValue);
        } else if (requestType == "RequestChat") {
            request = createRequestChat(requestValue);
        } else if (requestType == "RequestMusic") {
            request = createRequestMusic(requestValue);
        } else if (requestType == "RequestListening") {
            request = createRequestListening(requestValue);
        } else if (requestType == "RequestSystem") {
            request = createRequestSystem(requestValue);
        } else if (requestType == "RequestMovementType") {
            request = createRequestMovementType(requestValue);
        } else if (requestType == "RequestSinglePose") {
            request = createRequestMoveBody(requestValue);
        } else if (requestType == "RequestHeadOrientation") {
            request = createRequestHeadOrientation(requestValue);
        } else if (requestType == "RequestVelocity") {
            request = createRequestMoveVelocity(requestValue);
        } else {
            RCLCPP_WARN(node_->get_logger(), "Unknown request type: %s", requestType.c_str());
        }

        if (request != nullptr) {
            requests.push_back(request);
        }
    }

    return requests;
}

std::shared_ptr<RequestTalking> CBehaviorParser::createRequestTalking(const YAML::Node& value) {
    try {
        auto request = std::make_shared<RequestTalking>();

        if (value.IsScalar()) {
            request->text = value.as<std::string>();
        } else if (value.IsMap()) {
            request->text = valueOr<std::string>(value, "text", "");
            request->language = valueOr<std::string>(value, "language", "de");
            request->minDuration = valueOr(value, "minDuration", 0.0);
        }

        return request;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestTalking: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestChat> CBehaviorParser::createRequestChat(const YAML::Node& value) {
    try {
        auto request = std::make_shared<RequestChat>();

        if (value.IsScalar()) {
            request->text = value.as<std::string>();
        } else if (value.IsMap()) {
            request->text = valueOr<std::string>(value, "text", "");
            request->language = valueOr<std::string>(value, "language", "de");
            request->minDuration = valueOr(value, "minDuration", 0.0);
        }

        return request;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestChat: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestMusic> CBehaviorParser::createRequestMusic(const YAML::Node& value) {
    try {
        auto request = std::make_shared<RequestMusic>();

        if (value.IsScalar()) {
            request->song = value.as<std::string>();
        } else if (value.IsMap()) {
            request->song = valueOr<std::string>(value, "song", "");
            request->volume = valueOr(value, "volume", 0.8f);
            request->minDuration = valueOr(value, "minDuration", 0.0);
        }

        return request;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestMusic: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestListening> CBehaviorParser::createRequestListening(const YAML::Node& value) {
    try {
        auto request = std::make_shared<RequestListening>();

        if (value.IsScalar()) {
            request->active = value.as<bool>();
        } else if (value.IsMap()) {
            request->active = valueOr(value, "active", false);
            request->minDuration = valueOr(value, "minDuration", 0.0);
        }

        return request;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestListening: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestSystem> CBehaviorParser::createRequestSystem(const YAML::Node& value) {
    try {
        if (value.IsMap()) {
            auto request = std::make_shared<RequestSystem>();
            request->turnOffServoRelay = valueOr(value, "turnOffServoRelay", false);
            request->systemShutdown = valueOr(value, "systemShutdown", false);
            request->minDuration = valueOr(value, "minDuration", 0.0);
            return request;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestSystem: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestMovementType> CBehaviorParser::createRequestMovementType(const YAML::Node& value) {
    try {
        if (value.IsMap()) {
            brain::MovementRequest movementRequest;

            // Parse movement type from "name" field (or fallback to "type" for backward compatibility)
            std::string typeStr = "";
            if (value["name"] && value["name"].IsScalar()) {
                typeStr = value["name"].as<std::string>();
            } else if (value["type"] && value["type"].IsScalar()) {
                typeStr = value["type"].as<std::string>();
            }

            if (!typeStr.empty()) {
                // Look up movement type in map
                auto it = nameToMovementType.find(typeStr);
                if (it != nameToMovementType.end()) {
                    movementRequest.type = it->second;
                    movementRequest.name = typeStr;
                } else {
                    RCLCPP_WARN(node_->get_logger(), "Unknown movement type: %s", typeStr.c_str());
                }
            }

            // Parse direction if present
            if (value["direction"] && value["direction"].IsScalar()) {
                std::string directionStr = value["direction"].as<std::string>();
                if (directionStr == "CLOCKWISE") {
                    movementRequest.direction = brain::MovementRequest::CLOCKWISE;
                } else if (directionStr == "ANTICLOCKWISE") {
                    movementRequest.direction = brain::MovementRequest::ANTICLOCKWISE;
                } else {
                    RCLCPP_WARN(node_->get_logger(), "Unknown direction: %s", directionStr.c_str());
                }
            }
            if (value["duration_s"]) {
                movementRequest.duration_s = value["duration_s"].as<double>();
            }

            auto request = std::make_shared<RequestMovementType>();
            request->movementRequest = movementRequest;
            request->minDuration = valueOr(value, "minDuration", 0.0);
            return request;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestMovementType: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestSinglePose> CBehaviorParser::createRequestMoveBody(const YAML::Node& value) {
    try {
        if (value.IsMap()) {
            rumblex_interfaces::msg::Pose pose;

            // Parse position values (geometry_msgs/Vector3)
            if (value["position"] && value["position"].IsMap()) {
                const auto& position = value["position"];
                if (position["x"]) pose.position.x = position["x"].as<double>();
                if (position["y"]) pose.position.y = position["y"].as<double>();
                if (position["z"]) pose.position.z = position["z"].as<double>();
            }

            // Parse orientation values (Orientation with roll, pitch, yaw)
            if (value["orientation"] && value["orientation"].IsMap()) {
                const auto& orientation = value["orientation"];
                if (orientation["roll"]) pose.orientation.roll = orientation["roll"].as<double>();
                if (orientation["pitch"]) pose.orientation.pitch = orientation["pitch"].as<double>();
                if (orientation["yaw"]) pose.orientation.yaw = orientation["yaw"].as<double>();
            }

            auto request = std::make_shared<RequestSinglePose>();
            request->pose = pose;
            request->minDuration = valueOr(value, "minDuration", 0.0);
            return request;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestSinglePose: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestHeadOrientation> CBehaviorParser::createRequestHeadOrientation(
    const YAML::Node& value) {
    try {
        if (value.IsMap()) {
            rumblex_interfaces::msg::Orientation orientation;

            // Parse orientation values (roll, pitch, yaw)
            if (value["roll"]) orientation.roll = value["roll"].as<double>();
            if (value["pitch"]) orientation.pitch = value["pitch"].as<double>();
            if (value["yaw"]) orientation.yaw = value["yaw"].as<double>();

            auto request = std::make_shared<RequestHeadOrientation>();
            request->orientation = orientation;
            request->minDuration = valueOr(value, "minDuration", 0.0);
            return request;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestHeadOrientation: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestVelocity> CBehaviorParser::createRequestMoveVelocity(const YAML::Node& value) {
    try {
        if (value.IsMap()) {
            geometry_msgs::msg::Twist velocity;

            // Parse linear velocities
            if (value["linear"]) {
                const auto& linear = value["linear"];
                if (linear["x"]) velocity.linear.x = linear["x"].as<double>();
                if (linear["y"]) velocity.linear.y = linear["y"].as<double>();
                if (linear["z"]) velocity.linear.z = linear["z"].as<double>();
            }

            // Parse angular velocities
            if (value["angular"]) {
                const auto& angular = value["angular"];
                if (angular["x"]) velocity.angular.x = angular["x"].as<double>();
                if (angular["y"]) velocity.angular.y = angular["y"].as<double>();
                if (angular["z"]) velocity.angular.z = angular["z"].as<double>();
            }

            auto request = std::make_shared<RequestVelocity>();
            request->velocity = velocity;
            request->minDuration = valueOr(value, "minDuration", 0.0);
            return request;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestVelocity: %s", e.what());
    }

    return nullptr;
}

}  // namespace brain
