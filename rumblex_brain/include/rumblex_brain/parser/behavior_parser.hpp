/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include <functional>
#include <map>
#include <memory>
#include <nlohmann/json_fwd.hpp>
#include <optional>
#include <string>
#include <vector>

#include "rumblex_brain/requester/irequester.hpp"
#include "rumblex_interfaces/msg/joystick_request.hpp"
#include "rclcpp/rclcpp.hpp"
#include "requester/utility.hpp"

namespace brain {

struct Behavior {
    std::string name;
    std::vector<std::vector<std::shared_ptr<RequestBase>>> actionGroups;
};

struct BehaviorTrigger {
    std::string joystick;
    std::string voice;
};

/**
 * @brief Parser for behaviors.json file that creates Request objects
 * 
 * This class parses a JSON file containing behavior definitions and creates
 * appropriate Request objects (RequestTalking, RequestMusic, RequestMovementType, etc.)
 */
class CBehaviorParser {
   public:
    /**
     * @brief Constructor
     * @param node Shared pointer to ROS2 node for logging
     */
    explicit CBehaviorParser(rclcpp::Node::SharedPtr node);

    /**
     * @brief Destructor
     */
    ~CBehaviorParser() = default;

    /**
     * @brief Parse a behaviors.json file
     * @param filePath Path to the JSON file
     * @return true if parsing was successful, false otherwise
     */
    bool parseFile(const std::string& filePath);

    /**
     * @brief Parse a JSON string containing behavior definitions
     * @param jsonString JSON string to parse
     * @return true if parsing was successful, false otherwise
     */
    bool parseString(const std::string& jsonString);

    /**
     * @brief Get all behaviors parsed from the file
     * @return Vector of Behavior objects, each containing name and action groups
     */
    const std::vector<Behavior>& getBehaviors() const;

    /**
     * @brief Get a specific behavior by name
     * @param name Name of the behavior to retrieve
     * @return Optional reference to Behavior if found, nullopt otherwise
     */
    std::optional<std::reference_wrapper<const Behavior>> getBehavior(const std::string& name) const;

    /**
     * @brief Get behavior that matches the joystick request
     * @param msg Joystick request message
     * @return Optional reference to Behavior if a matching trigger is found, nullopt otherwise
     */
    std::optional<std::reference_wrapper<const Behavior>> getBehaviorForJoystickRequest(
        const rumblex_interfaces::msg::JoystickRequest& msg) const;

    /**
     * @brief Get behavior that matches the voice command
     * @param voiceCommand Voice command string
     * @return Optional reference to Behavior if a matching trigger is found, nullopt otherwise
     */
    std::optional<std::reference_wrapper<const Behavior>> getBehaviorForVoiceRequest(
        const std::string& voiceCommand) const;

   private:
    /**
     * @brief Parse multiple behaviors from JSON
     * @param behaviorsJson JSON value containing behaviors array
     * @return true if parsing was successful
     */
    bool parseBehaviors(const nlohmann::json& behaviorsJson);

    /**
     * @brief Parse a single behavior
     * @param behaviorJson JSON value containing a behavior object
     * @return true if parsing was successful
     */
    bool parseSingleBehavior(const nlohmann::json& behaviorJson);

    /**
     * @brief Parse a single action group
     * @param actionJson JSON value containing a single action object
     * @return Vector of Request objects for this action group
     */
    std::vector<std::shared_ptr<RequestBase>> parseActionGroup(const nlohmann::json& actionJson);

    /**
     * @brief Create RequestTalking from JSON value
     * @param value JSON value (string or object)
     * @return Shared pointer to RequestTalking object, or nullptr if parsing failed
     */
    std::shared_ptr<RequestTalking> createRequestTalking(const nlohmann::json& value);

    /**
     * @brief Create RequestChat from JSON value
     * @param value JSON value (string or object)
     * @return Shared pointer to RequestChat object, or nullptr if parsing failed
     */
    std::shared_ptr<RequestChat> createRequestChat(const nlohmann::json& value);

    /**
     * @brief Create RequestMusic from JSON value
     * @param value JSON value (string or object)
     * @return Shared pointer to RequestMusic object, or nullptr if parsing failed
     */
    std::shared_ptr<RequestMusic> createRequestMusic(const nlohmann::json& value);

    /**
     * @brief Create RequestListening from JSON value
     * @param value JSON value (boolean or object)
     * @return Shared pointer to RequestListening object, or nullptr if parsing failed
     */
    std::shared_ptr<RequestListening> createRequestListening(const nlohmann::json& value);

    /**
     * @brief Create RequestSystem from JSON value
     * @param value JSON value (object)
     * @return Shared pointer to RequestSystem object, or nullptr if parsing failed
     */
    std::shared_ptr<RequestSystem> createRequestSystem(const nlohmann::json& value);

    /**
     * @brief Create RequestMovementType from JSON value
     * @param value JSON value (object)
     * @return Shared pointer to RequestMovementType object, or nullptr if parsing failed
     */
    std::shared_ptr<RequestMovementType> createRequestMovementType(const nlohmann::json& value);

    /**
     * @brief Create RequestSinglePose from JSON value
     * @param value JSON value (object)
     * @return Shared pointer to RequestSinglePose object, or nullptr if parsing failed
     */
    std::shared_ptr<RequestSinglePose> createRequestMoveBody(const nlohmann::json& value);

    /**
     * @brief Create RequestHeadOrientation from JSON value
     * @param value JSON value (object)
     * @return Shared pointer to RequestHeadOrientation object, or nullptr if parsing failed
     */
    std::shared_ptr<RequestHeadOrientation> createRequestHeadOrientation(const nlohmann::json& value);

    /**
     * @brief Create RequestVelocity from JSON value
     * @param value JSON value (object)
     * @return Shared pointer to RequestVelocity object, or nullptr if parsing failed
     */
    std::shared_ptr<RequestVelocity> createRequestMoveVelocity(const nlohmann::json& value);

    rclcpp::Node::SharedPtr node_;
    std::vector<Behavior> behaviors_;
    std::map<std::string, BehaviorTrigger> behaviorTriggers_;  // Map behavior name to triggers
};

}  // namespace brain
