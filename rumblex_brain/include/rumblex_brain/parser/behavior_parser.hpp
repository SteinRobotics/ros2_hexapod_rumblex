/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "requester/utility.hpp"
#include "rumblex_brain/requester/irequester.hpp"
#include "rumblex_interfaces/msg/joystick_request.hpp"

namespace YAML {
class Node;
}

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
 * @brief Parser for behaviors.yaml file that creates Request objects
 * 
 * This class parses a YAML file containing behavior definitions and creates
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
     * @brief Parse a behaviors.yaml file
     * @param filePath Path to the YAML file
     * @return true if parsing was successful, false otherwise
     */
    bool parseFile(const std::string& filePath);

    /**
     * @brief Parse a YAML string containing behavior definitions
     * @param yamlString YAML string to parse
     * @return true if parsing was successful, false otherwise
     */
    bool parseString(const std::string& yamlString);

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
     * @brief Parse multiple behaviors from YAML
     * @param behaviorsNode YAML value containing behaviors array
     * @return true if parsing was successful
     */
    bool parseBehaviors(const YAML::Node& behaviorsNode);

    /**
     * @brief Parse a single behavior
     * @param behaviorNode YAML value containing a behavior object
     * @return true if parsing was successful
     */
    bool parseSingleBehavior(const YAML::Node& behaviorNode);

    /**
     * @brief Parse a single action group
     * @param actionNode YAML value containing a single action object
     * @return Vector of Request objects for this action group
     */
    std::vector<std::shared_ptr<RequestBase>> parseActionGroup(const YAML::Node& actionNode);

    /**
     * @brief Create RequestTalking from YAML value
     * @param value YAML value (string or object)
     * @return Shared pointer to RequestTalking object, or nullptr if parsing failed
     */
    std::shared_ptr<RequestTalking> createRequestTalking(const YAML::Node& value);

    /**
     * @brief Create RequestChat from YAML value
     * @param value YAML value (string or object)
     * @return Shared pointer to RequestChat object, or nullptr if parsing failed
     */
    std::shared_ptr<RequestChat> createRequestChat(const YAML::Node& value);

    /**
     * @brief Create RequestMusic from YAML value
     * @param value YAML value (string or object)
     * @return Shared pointer to RequestMusic object, or nullptr if parsing failed
     */
    std::shared_ptr<RequestMusic> createRequestMusic(const YAML::Node& value);

    /**
     * @brief Create RequestListening from YAML value
     * @param value YAML value (boolean or object)
     * @return Shared pointer to RequestListening object, or nullptr if parsing failed
     */
    std::shared_ptr<RequestListening> createRequestListening(const YAML::Node& value);

    /**
     * @brief Create RequestSystem from YAML value
     * @param value YAML value (object)
     * @return Shared pointer to RequestSystem object, or nullptr if parsing failed
     */
    std::shared_ptr<RequestSystem> createRequestSystem(const YAML::Node& value);

    /**
     * @brief Create RequestMovementType from YAML value
     * @param value YAML value (object)
     * @return Shared pointer to RequestMovementType object, or nullptr if parsing failed
     */
    std::shared_ptr<RequestMovementType> createRequestMovementType(const YAML::Node& value);

    /**
     * @brief Create RequestSinglePose from YAML value
     * @param value YAML value (object)
     * @return Shared pointer to RequestSinglePose object, or nullptr if parsing failed
     */
    std::shared_ptr<RequestSinglePose> createRequestMoveBody(const YAML::Node& value);

    /**
     * @brief Create RequestHeadOrientation from YAML value
     * @param value YAML value (object)
     * @return Shared pointer to RequestHeadOrientation object, or nullptr if parsing failed
     */
    std::shared_ptr<RequestHeadOrientation> createRequestHeadOrientation(const YAML::Node& value);

    /**
     * @brief Create RequestVelocity from YAML value
     * @param value YAML value (object)
     * @return Shared pointer to RequestVelocity object, or nullptr if parsing failed
     */
    std::shared_ptr<RequestVelocity> createRequestMoveVelocity(const YAML::Node& value);

    rclcpp::Node::SharedPtr node_;
    std::vector<Behavior> behaviors_;
    std::map<std::string, BehaviorTrigger> behaviorTriggers_;  // Map behavior name to triggers
};

}  // namespace brain
