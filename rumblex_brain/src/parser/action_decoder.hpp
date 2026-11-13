#pragma once
#include "rclcpp/rclcpp.hpp"
#include "requester/irequester.hpp"
#include "requester/utility.hpp"

namespace YAML {
class Node;
}
namespace brain::sequence_detail {
class ActionDecoder {
   public:
    explicit ActionDecoder(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {
    }
    std::vector<std::shared_ptr<RequestBase>> parseActionGroup(const YAML::Node& action);

   private:
    std::shared_ptr<RequestTalking> createRequestTalking(const YAML::Node& value);
    std::shared_ptr<RequestChat> createRequestChat(const YAML::Node& value);
    std::shared_ptr<RequestMusic> createRequestMusic(const YAML::Node& value);
    std::shared_ptr<RequestListening> createRequestListening(const YAML::Node& value);
    std::shared_ptr<RequestSystem> createRequestSystem(const YAML::Node& value);
    std::shared_ptr<RequestMovementType> createRequestMovementType(const YAML::Node& value);
    std::shared_ptr<RequestVelocity> createRequestMoveVelocity(const YAML::Node& value);
    rclcpp::Node::SharedPtr node_;
};
}  // namespace brain::sequence_detail
