/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "rumblex_brain/parser/behavior_parser.hpp"

#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace brain {

CBehaviorParser::CBehaviorParser(rclcpp::Node::SharedPtr node) : node_(node) {
}

bool CBehaviorParser::parseFile(const std::string& filePath) {
    try {
        std::ifstream file(filePath);
        if (!file.is_open()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open behavior file: %s", filePath.c_str());
            return false;
        }

        json jsonData;
        file >> jsonData;
        file.close();

        return parseString(jsonData.dump());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Exception while parsing file: %s", e.what());
        return false;
    }
}

bool CBehaviorParser::parseString(const std::string& jsonString) {
    try {
        json jsonData = json::parse(jsonString);

        if (jsonData.contains("behaviors") && jsonData["behaviors"].is_array()) {
            return parseBehaviors(jsonData["behaviors"]);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Invalid JSON format: expected 'behaviors' array");
            return false;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Exception while parsing JSON string: %s", e.what());
        return false;
    }
}

bool CBehaviorParser::parseBehaviors(const json& behaviorsArray) {
    if (!behaviorsArray.is_array()) {
        RCLCPP_ERROR(node_->get_logger(), "Expected behaviors to be an array");
        return false;
    }

    behaviors_.clear();

    for (const auto& behaviorJson : behaviorsArray) {
        if (!parseSingleBehavior(behaviorJson)) {
            RCLCPP_WARN(node_->get_logger(), "Failed to parse one behavior, continuing with others");
        }
    }

    return !behaviors_.empty();
}

bool CBehaviorParser::parseSingleBehavior(const json& j) {
    Behavior behavior;

    // Parse behavior name
    if (j.contains("name") && j["name"].is_string()) {
        behavior.name = j["name"].get<std::string>();
    } else {
        RCLCPP_WARN(node_->get_logger(), "Behavior name not found or invalid");
        behavior.name = "UNKNOWN";
    }

    // Parse triggers
    BehaviorTrigger trigger;
    if (j.contains("trigger") && j["trigger"].is_object()) {
        const auto& triggerJson = j["trigger"];
        if (triggerJson.contains("joystick") && triggerJson["joystick"].is_string()) {
            trigger.joystick = triggerJson["joystick"].get<std::string>();
        }
        if (triggerJson.contains("voice") && triggerJson["voice"].is_string()) {
            trigger.voice = triggerJson["voice"].get<std::string>();
        }
        behaviorTriggers_[behavior.name] = trigger;
    }

    // Parse actions
    if (j.contains("actions") && j["actions"].is_array()) {
        const auto& actionsJson = j["actions"];

        for (const auto& actionJson : actionsJson) {
            auto requests = parseActionGroup(actionJson);
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

std::vector<std::shared_ptr<RequestBase>> CBehaviorParser::parseActionGroup(const json& actionObject) {
    std::vector<std::shared_ptr<RequestBase>> requests;

    // Iterate through all keys in the action object
    for (auto it = actionObject.begin(); it != actionObject.end(); ++it) {
        const std::string& requestType = it.key();
        const json& requestValue = it.value();

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

std::shared_ptr<RequestTalking> CBehaviorParser::createRequestTalking(const json& jsonValue) {
    try {
        auto request = std::make_shared<RequestTalking>();

        if (jsonValue.is_string()) {
            request->text = jsonValue.get<std::string>();
        } else if (jsonValue.is_object()) {
            request->text = jsonValue.value("text", "");
            request->language = jsonValue.value("language", "de");
            request->minDuration = jsonValue.value("minDuration", 0.0);
        }

        return request;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestTalking: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestChat> CBehaviorParser::createRequestChat(const json& jsonValue) {
    try {
        auto request = std::make_shared<RequestChat>();

        if (jsonValue.is_string()) {
            request->text = jsonValue.get<std::string>();
        } else if (jsonValue.is_object()) {
            request->text = jsonValue.value("text", "");
            request->language = jsonValue.value("language", "de");
            request->minDuration = jsonValue.value("minDuration", 0.0);
        }

        return request;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestChat: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestMusic> CBehaviorParser::createRequestMusic(const json& jsonValue) {
    try {
        auto request = std::make_shared<RequestMusic>();

        if (jsonValue.is_string()) {
            request->song = jsonValue.get<std::string>();
        } else if (jsonValue.is_object()) {
            request->song = jsonValue.value("song", "");
            request->volume = jsonValue.value("volume", 0.8f);
            request->minDuration = jsonValue.value("minDuration", 0.0);
        }

        return request;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestMusic: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestListening> CBehaviorParser::createRequestListening(const json& jsonValue) {
    try {
        auto request = std::make_shared<RequestListening>();

        if (jsonValue.is_boolean()) {
            request->active = jsonValue.get<bool>();
        } else if (jsonValue.is_object()) {
            request->active = jsonValue.value("active", false);
            request->minDuration = jsonValue.value("minDuration", 0.0);
        }

        return request;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestListening: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestSystem> CBehaviorParser::createRequestSystem(const json& jsonValue) {
    try {
        if (jsonValue.is_object()) {
            auto request = std::make_shared<RequestSystem>();
            request->turnOffServoRelay = jsonValue.value("turnOffServoRelay", false);
            request->systemShutdown = jsonValue.value("systemShutdown", false);
            request->minDuration = jsonValue.value("minDuration", 0.0);
            return request;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestSystem: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestMovementType> CBehaviorParser::createRequestMovementType(const json& jsonValue) {
    try {
        if (jsonValue.is_object()) {
            rumblex_interfaces::msg::MovementRequest movementRequest;

            // Parse movement type from "name" field (or fallback to "type" for backward compatibility)
            std::string typeStr = "";
            if (jsonValue.contains("name") && jsonValue["name"].is_string()) {
                typeStr = jsonValue["name"].get<std::string>();
            } else if (jsonValue.contains("type") && jsonValue["type"].is_string()) {
                typeStr = jsonValue["type"].get<std::string>();
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
            if (jsonValue.contains("direction") && jsonValue["direction"].is_string()) {
                std::string directionStr = jsonValue["direction"].get<std::string>();
                if (directionStr == "CLOCKWISE") {
                    movementRequest.direction = rumblex_interfaces::msg::MovementRequest::CLOCKWISE;
                } else if (directionStr == "ANTICLOCKWISE") {
                    movementRequest.direction = rumblex_interfaces::msg::MovementRequest::ANTICLOCKWISE;
                } else {
                    RCLCPP_WARN(node_->get_logger(), "Unknown direction: %s", directionStr.c_str());
                }
            }
            if (jsonValue.contains("duration_s")) {
                movementRequest.duration_s = jsonValue["duration_s"].get<double>();
            }

            auto request = std::make_shared<RequestMovementType>();
            request->movementRequest = movementRequest;
            request->minDuration = jsonValue.value("minDuration", 0.0);
            return request;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestMovementType: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestSinglePose> CBehaviorParser::createRequestMoveBody(const json& jsonValue) {
    try {
        if (jsonValue.is_object()) {
            rumblex_interfaces::msg::Pose pose;

            // Parse position values (geometry_msgs/Vector3)
            if (jsonValue.contains("position") && jsonValue["position"].is_object()) {
                const auto& position = jsonValue["position"];
                if (position.contains("x")) pose.position.x = position["x"].get<double>();
                if (position.contains("y")) pose.position.y = position["y"].get<double>();
                if (position.contains("z")) pose.position.z = position["z"].get<double>();
            }

            // Parse orientation values (Orientation with roll, pitch, yaw)
            if (jsonValue.contains("orientation") && jsonValue["orientation"].is_object()) {
                const auto& orientation = jsonValue["orientation"];
                if (orientation.contains("roll")) pose.orientation.roll = orientation["roll"].get<double>();
                if (orientation.contains("pitch"))
                    pose.orientation.pitch = orientation["pitch"].get<double>();
                if (orientation.contains("yaw")) pose.orientation.yaw = orientation["yaw"].get<double>();
            }

            auto request = std::make_shared<RequestSinglePose>();
            request->pose = pose;
            request->minDuration = jsonValue.value("minDuration", 0.0);
            return request;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestSinglePose: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestHeadOrientation> CBehaviorParser::createRequestHeadOrientation(const json& jsonValue) {
    try {
        if (jsonValue.is_object()) {
            rumblex_interfaces::msg::Orientation orientation;

            // Parse orientation values (roll, pitch, yaw)
            if (jsonValue.contains("roll")) orientation.roll = jsonValue["roll"].get<double>();
            if (jsonValue.contains("pitch")) orientation.pitch = jsonValue["pitch"].get<double>();
            if (jsonValue.contains("yaw")) orientation.yaw = jsonValue["yaw"].get<double>();

            auto request = std::make_shared<RequestHeadOrientation>();
            request->orientation = orientation;
            request->minDuration = jsonValue.value("minDuration", 0.0);
            return request;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestHeadOrientation: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestVelocity> CBehaviorParser::createRequestMoveVelocity(const json& jsonValue) {
    try {
        if (jsonValue.is_object()) {
            geometry_msgs::msg::Twist velocity;

            // Parse linear velocities
            if (jsonValue.contains("linear")) {
                const auto& linear = jsonValue["linear"];
                if (linear.contains("x")) velocity.linear.x = linear["x"].get<double>();
                if (linear.contains("y")) velocity.linear.y = linear["y"].get<double>();
                if (linear.contains("z")) velocity.linear.z = linear["z"].get<double>();
            }

            // Parse angular velocities
            if (jsonValue.contains("angular")) {
                const auto& angular = jsonValue["angular"];
                if (angular.contains("x")) velocity.angular.x = angular["x"].get<double>();
                if (angular.contains("y")) velocity.angular.y = angular["y"].get<double>();
                if (angular.contains("z")) velocity.angular.z = angular["z"].get<double>();
            }

            auto request = std::make_shared<RequestVelocity>();
            request->velocity = velocity;
            request->minDuration = jsonValue.value("minDuration", 0.0);
            return request;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestVelocity: %s", e.what());
    }

    return nullptr;
}

}  // namespace brain
