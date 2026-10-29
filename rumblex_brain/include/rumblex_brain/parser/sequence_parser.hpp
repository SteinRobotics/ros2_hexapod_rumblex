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
struct Sequence {
    std::string name;
    std::vector<std::vector<std::shared_ptr<RequestBase>>> actionGroups;
};
struct SequenceTrigger {
    std::string joystick;
    std::string voice;
};
class CSequenceParser {
   public:
    explicit CSequenceParser(rclcpp::Node::SharedPtr node);
    // All loads are transactional. Directory loading is sorted and nonrecursive.
    bool parseFile(const std::string& filePath);
    bool parseDirectory(const std::string& directoryPath);
    // Supports loops, but includes require a source file for relative paths.
    bool parseString(const std::string& yamlString);
    const std::vector<Sequence>& getSequences() const;
    std::optional<std::reference_wrapper<const Sequence>> getSequence(const std::string& name) const;
    std::optional<std::reference_wrapper<const Sequence>> getSequenceForJoystickRequest(
        const rumblex_interfaces::msg::JoystickRequest& msg) const;
    std::optional<std::reference_wrapper<const Sequence>> getSequenceForVoiceRequest(
        const std::string& voiceCommand) const;

   private:
    bool parseSequences(const YAML::Node& sequences);
    bool parseSingleSequence(const YAML::Node& sequence);
    std::vector<std::shared_ptr<RequestBase>> parseActionGroup(const YAML::Node& action);
    std::shared_ptr<RequestTalking> createRequestTalking(const YAML::Node& value);
    std::shared_ptr<RequestChat> createRequestChat(const YAML::Node& value);
    std::shared_ptr<RequestMusic> createRequestMusic(const YAML::Node& value);
    std::shared_ptr<RequestListening> createRequestListening(const YAML::Node& value);
    std::shared_ptr<RequestSystem> createRequestSystem(const YAML::Node& value);
    std::shared_ptr<RequestMovementType> createRequestMovementType(const YAML::Node& value);
    std::shared_ptr<RequestVelocity> createRequestMoveVelocity(const YAML::Node& value);
    rclcpp::Node::SharedPtr node_;
    std::vector<Sequence> sequences_;
    std::map<std::string, SequenceTrigger> sequenceTriggers_;
};
}  // namespace brain
