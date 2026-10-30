/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "rumblex_brain/parser/sequence_parser.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>
#include <set>
#include <stdexcept>

#include "rumblex_utils/body_pose.hpp"

namespace brain {

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
namespace {
// Defaults apply only to omitted fields; malformed explicit values remain errors.
template <typename T>
T valueOr(const YAML::Node& node, const char* key, const T& fallback) {
    return node[key] ? node[key].as<T>() : fallback;
}
void keys(const YAML::Node& node, std::initializer_list<std::string> allowed) {
    if (!node.IsMap()) throw std::runtime_error("Expected mapping");
    std::set<std::string> seen;
    for (const auto& entry : node) {
        const auto key = entry.first.as<std::string>();
        if (!seen.insert(key).second || std::find(allowed.begin(), allowed.end(), key) == allowed.end())
            throw std::runtime_error("Unknown or duplicate field: " + key);
    }
}
void parseVector(const YAML::Node& node, geometry_msgs::msg::Vector3& vector) {
    keys(node, {"x", "y", "z"});
    vector.x = valueOr(node, "x", 0.0);
    vector.y = valueOr(node, "y", 0.0);
    vector.z = valueOr(node, "z", 0.0);
}
void parseOrientation(const YAML::Node& node, rumblex_interfaces::msg::Orientation& orientation) {
    keys(node, {"roll", "pitch", "yaw"});
    orientation.roll = valueOr(node, "roll", 0.0);
    orientation.pitch = valueOr(node, "pitch", 0.0);
    orientation.yaw = valueOr(node, "yaw", 0.0);
}
rumblex_interfaces::msg::BodyPose parseBodyPose(const YAML::Node& value) {
    keys(value, {"duration_s", "torso_pose", "head_orientation", "toe_positions"});
    rumblex_interfaces::msg::BodyPose pose;
    if (value["torso_pose"]) {
        pose.has_torso_pose = true;
        const auto torso = value["torso_pose"];
        keys(torso, {"position", "orientation"});
        if (torso["position"]) parseVector(torso["position"], pose.torso_pose.position);
        if (torso["orientation"]) parseOrientation(torso["orientation"], pose.torso_pose.orientation);
    }
    if (value["head_orientation"]) {
        pose.has_head_orientation = true;
        parseOrientation(value["head_orientation"], pose.head_orientation);
    }
    if (value["toe_positions"]) {
        const auto toes = value["toe_positions"];
        if (!toes.IsSequence() || toes.size() != pose.toe_positions.size())
            throw std::runtime_error("toe_positions requires all six toes in leg order");
        pose.has_toe_positions = true;
        for (size_t i = 0; i < toes.size(); ++i) {
            if (!toes[i].IsMap() || toes[i].size() != 3)
                throw std::runtime_error("Every toe requires x, y and z");
            parseVector(toes[i], pose.toe_positions[i]);
        }
    }
    if (!rumblex_geometry::hasBodyPose(pose) || !rumblex_geometry::validBodyPose(pose))
        throw std::runtime_error("body_pose requires at least one finite component");
    return pose;
}
// Emit directly into one bounded catalog; includes and loops share the same budget.
void expandActions(const YAML::Node& actions, const std::filesystem::path& source,
                   std::vector<std::filesystem::path>& stack, YAML::Node& expanded, size_t& groups,
                   unsigned loopDepth = 0) {
    if (!actions.IsSequence() || actions.size() == 0)
        throw std::runtime_error("Expected nonempty actions sequence in " + source.string());
    for (const auto& action : actions) {
        if (!action.IsMap() || action.size() == 0)
            throw std::runtime_error("Expected nonempty action mapping");
        if (action["include"]) {
            keys(action, {"include"});
            if (source.empty()) throw std::runtime_error("Snippet includes require a source file");
            const auto path =
                std::filesystem::canonical(source.parent_path() / action["include"].as<std::string>());
            if (std::find(stack.begin(), stack.end(), path) != stack.end())
                throw std::runtime_error("Circular snippet include: " + path.string());
            if (stack.size() >= 64) throw std::runtime_error("Snippet nesting exceeds 64 files");
            stack.push_back(path);
            const auto snippet = YAML::LoadFile(path.string());
            keys(snippet, {"actions"});
            expandActions(snippet["actions"], path, stack, expanded, groups, loopDepth);
            stack.pop_back();
        } else if (action["loop"]) {
            keys(action, {"loop"});
            const auto loop = action["loop"];
            keys(loop, {"times", "actions"});
            const auto count = loop["times"].as<int>();
            if (count < 1 || count > 1000 || loopDepth >= 16)
                throw std::runtime_error("Loop requires 1..1000 repetitions and at most 16 nested loops");
            for (int i = 0; i < count; ++i)
                expandActions(loop["actions"], source, stack, expanded, groups, loopDepth + 1);
        } else {
            if (++groups > 10000) throw std::runtime_error("Catalog exceeds 10000 expanded action groups");
            expanded.push_back(action);
        }
    }
}
YAML::Node loadSequences(const YAML::Node& document, const std::filesystem::path& source, size_t& groups) {
    YAML::Node sequences(YAML::NodeType::Sequence);
    if (document.IsMap() && (document["sequences"] || document["behaviors"])) {
        if (document["sequences"] && document["behaviors"])
            throw std::runtime_error("Cannot mix sequences and legacy behaviors wrappers");
        keys(document, {"sequences", "behaviors"});
        const auto list = document["sequences"] ? document["sequences"] : document["behaviors"];
        if (!list.IsSequence()) throw std::runtime_error("Expected sequences list");
        for (const auto& sequence : list) sequences.push_back(YAML::Clone(sequence));
    } else if (document.IsMap() && document["name"]) {
        sequences.push_back(YAML::Clone(document));
    } else {
        throw std::runtime_error("Expected a named sequence or sequences list");
    }
    for (auto sequence : sequences) {
        std::vector<std::filesystem::path> stack;
        if (!source.empty()) stack.push_back(source);
        YAML::Node expanded(YAML::NodeType::Sequence);
        expandActions(sequence["actions"], source, stack, expanded, groups);
        sequence["actions"] = expanded;
    }
    return sequences;
}
YAML::Node loadSequences(const std::filesystem::path& file, size_t& groups) {
    const auto path = std::filesystem::canonical(file);
    return loadSequences(YAML::LoadFile(path.string()), path, groups);
}
}  // namespace

CSequenceParser::CSequenceParser(rclcpp::Node::SharedPtr node) : node_(node) {
}

bool CSequenceParser::parseFile(const std::string& filePath) {
    try {
        CSequenceParser parsed(node_);
        size_t groups = 0;
        const auto sequences = loadSequences(filePath, groups);
        if (!parsed.parseSequences(sequences) || parsed.sequences_.size() != sequences.size()) return false;
        sequences_ = std::move(parsed.sequences_);
        sequenceTriggers_ = std::move(parsed.sequenceTriggers_);
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Exception while parsing file: %s", e.what());
        return false;
    }
}

bool CSequenceParser::parseDirectory(const std::string& directoryPath) {
    try {
        std::vector<std::filesystem::path> files;
        for (const auto& entry : std::filesystem::directory_iterator(directoryPath)) {
            const auto extension = entry.path().extension();
            if (entry.is_regular_file() && (extension == ".yaml" || extension == ".yml")) {
                files.push_back(entry.path());
            }
        }
        std::sort(files.begin(), files.end());
        YAML::Node catalog(YAML::NodeType::Sequence);
        size_t groups = 0;
        for (const auto& file : files) {
            for (const auto& sequence : loadSequences(file, groups)) catalog.push_back(sequence);
        }
        CSequenceParser parsed(node_);
        if (!parsed.parseSequences(catalog) || parsed.sequences_.size() != catalog.size()) return false;
        sequences_ = std::move(parsed.sequences_);
        sequenceTriggers_ = std::move(parsed.sequenceTriggers_);
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load sequence directory '%s': %s", directoryPath.c_str(),
                     e.what());
        return false;
    }
}

bool CSequenceParser::parseString(const std::string& yamlString) {
    try {
        CSequenceParser parsed(node_);
        size_t groups = 0;
        const auto document = YAML::Load(yamlString);
        if (document.IsMap() && document["behaviors"])
            RCLCPP_WARN(node_->get_logger(), "Legacy behaviors wrapper is deprecated; use sequences");
        const auto sequences = loadSequences(document, {}, groups);
        if (!parsed.parseSequences(sequences)) return false;
        sequences_ = std::move(parsed.sequences_);
        sequenceTriggers_ = std::move(parsed.sequenceTriggers_);
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Exception while parsing YAML string: %s", e.what());
        return false;
    }
}

bool CSequenceParser::parseSequences(const YAML::Node& sequencesArray) {
    if (!sequencesArray.IsSequence()) {
        RCLCPP_ERROR(node_->get_logger(), "Expected sequences to be an array");
        return false;
    }

    sequences_.clear();
    sequenceTriggers_.clear();

    for (const auto& sequenceNode : sequencesArray) {
        if (!parseSingleSequence(sequenceNode)) {
            return false;
        }
    }

    return !sequences_.empty();
}

bool CSequenceParser::parseSingleSequence(const YAML::Node& j) {
    Sequence sequence;

    if (!j.IsMap()) {
        RCLCPP_WARN(node_->get_logger(), "Expected sequence to be a mapping");
        return false;
    }

    keys(j, {"name", "trigger", "actions"});
    // Parse sequence name
    if (j["name"] && j["name"].IsScalar()) {
        sequence.name = j["name"].as<std::string>();
    } else {
        RCLCPP_WARN(node_->get_logger(), "Sequence name not found or invalid");
        return false;
    }

    if (sequence.name.empty()) return false;
    if (getSequence(sequence.name)) {
        throw std::runtime_error("Duplicate sequence name: " + sequence.name);
    }

    if (j["trigger"]) keys(j["trigger"], {"joystick", "voice"});
    // Parse triggers
    SequenceTrigger trigger;
    if (j["trigger"] && j["trigger"].IsMap()) {
        const auto& triggerNode = j["trigger"];
        if (triggerNode["joystick"] && triggerNode["joystick"].IsScalar()) {
            trigger.joystick = triggerNode["joystick"].as<std::string>();
        }
        if (triggerNode["voice"] && triggerNode["voice"].IsScalar()) {
            trigger.voice = triggerNode["voice"].as<std::string>();
        }
    }

    // Parse actions
    if (j["actions"] && j["actions"].IsSequence()) {
        const auto& actionsNode = j["actions"];

        for (const auto& actionNode : actionsNode) {
            std::vector<std::shared_ptr<RequestBase>> requests;
            try {
                requests = parseActionGroup(actionNode);
            } catch (const std::exception& error) {
                throw std::runtime_error("Sequence " + sequence.name + ", action " +
                                         std::to_string(sequence.actionGroups.size() + 1) + ": " +
                                         error.what());
            }
            if (!requests.empty()) {
                sequence.actionGroups.push_back(requests);
            }
        }

        if (sequence.actionGroups.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "No valid action groups found for sequence: %s",
                         sequence.name.c_str());
            return false;
        }
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Actions array not found for sequence: %s", sequence.name.c_str());
        return false;
    }

    sequenceTriggers_[sequence.name] = trigger;
    sequences_.push_back(sequence);
    RCLCPP_INFO(node_->get_logger(), "Parsed sequence '%s' with %zu action groups", sequence.name.c_str(),
                sequence.actionGroups.size());

    return true;
}

const std::vector<Sequence>& CSequenceParser::getSequences() const {
    return sequences_;
}

std::optional<std::reference_wrapper<const Sequence>> CSequenceParser::getSequence(
    const std::string& name) const {
    for (const auto& sequence : sequences_) {
        if (sequence.name == name) {
            return std::cref(sequence);
        }
    }
    return std::nullopt;
}

std::optional<std::reference_wrapper<const Sequence>> CSequenceParser::getSequenceForJoystickRequest(
    const rumblex_interfaces::msg::JoystickRequest& msg) const {
    // Check each sequence's joystick trigger
    for (const auto& sequence : sequences_) {
        const auto& trigger = sequenceTriggers_.find(sequence.name);
        if (trigger == sequenceTriggers_.end()) {
            continue;
        }

        const std::string& joystickTrigger = trigger->second.joystick;

        // Match button triggers
        if (joystickTrigger == "button_a" && msg.button_a) return std::cref(sequence);
        if (joystickTrigger == "button_b" && msg.button_b) return std::cref(sequence);
        if (joystickTrigger == "button_x" && msg.button_x) return std::cref(sequence);
        if (joystickTrigger == "button_y" && msg.button_y) return std::cref(sequence);
        if (joystickTrigger == "button_l1" && msg.button_l1) return std::cref(sequence);
        if (joystickTrigger == "button_l2" && msg.button_l2) return std::cref(sequence);
        if (joystickTrigger == "button_r1" && msg.button_r1) return std::cref(sequence);
        if (joystickTrigger == "button_r2" && msg.button_r2) return std::cref(sequence);
        if (joystickTrigger == "button_select" && msg.button_select) return std::cref(sequence);
        if (joystickTrigger == "button_start" && msg.button_start) return std::cref(sequence);
        if (joystickTrigger == "button_home" && msg.button_home) return std::cref(sequence);

        // Match dpad triggers
        if (joystickTrigger == "dpad_vertical_up" && msg.dpad_vertical == 1) return std::cref(sequence);
        if (joystickTrigger == "dpad_vertical_down" && msg.dpad_vertical == -1) return std::cref(sequence);
        if (joystickTrigger == "dpad_horizontal_left" && msg.dpad_horizontal == -1)
            return std::cref(sequence);
        if (joystickTrigger == "dpad_horizontal_right" && msg.dpad_horizontal == 1)
            return std::cref(sequence);
    }

    return std::nullopt;
}

std::optional<std::reference_wrapper<const Sequence>> CSequenceParser::getSequenceForVoiceRequest(
    const std::string& voiceCommand) const {
    // Check each sequence's voice trigger
    for (const auto& sequence : sequences_) {
        const auto& trigger = sequenceTriggers_.find(sequence.name);
        if (trigger == sequenceTriggers_.end()) {
            continue;
        }

        const std::string& voiceTrigger = trigger->second.voice;
        if (voiceTrigger == voiceCommand) {
            return std::cref(sequence);
        }
    }

    return std::nullopt;
}

std::vector<std::shared_ptr<RequestBase>> CSequenceParser::parseActionGroup(const YAML::Node& action) {
    static const std::map<std::string, std::string> aliases = {
        {"say", "RequestTalking"},         {"chat", "RequestChat"},     {"music", "RequestMusic"},
        {"listening", "RequestListening"}, {"system", "RequestSystem"}, {"movement", "RequestMovementType"},
        {"velocity", "RequestVelocity"}};
    std::map<std::string, YAML::Node> ordered;
    for (const auto& entry : action) {
        auto key = entry.first.as<std::string>();
        if (aliases.contains(key))
            key = aliases.at(key);
        else if (key.starts_with("Request"))
            RCLCPP_WARN(node_->get_logger(), "Legacy action %s is deprecated", key.c_str());
        if (!ordered.emplace(key, entry.second).second)
            throw std::runtime_error("Duplicate action or alias: " + key);
    }
    auto movement = std::shared_ptr<RequestMovementType>();
    if (ordered.contains("body_pose")) {
        if (ordered.contains("RequestMovementType") || ordered.contains("RequestSinglePose") ||
            ordered.contains("RequestHeadOrientation"))
            throw std::runtime_error("body_pose conflicts with another movement target");
        const auto value = ordered.at("body_pose");
        movement = std::make_shared<RequestMovementType>();
        movement->movementRequest.body_pose = parseBodyPose(value);
        movement->movementRequest.type = MovementRequest::SINGLE_POSE;
        movement->movementRequest.name = "SINGLE_POSE";
        const double duration = valueOr(value, "duration_s", 1.0);
        if (!std::isfinite(duration) || duration < 1.0)
            throw std::runtime_error("body_pose duration_s must be finite and at least 1 second");
        movement->movementRequest.duration_s = duration * s;
        ordered.erase("body_pose");
        ordered.emplace("RequestMovementType", YAML::Node());
    } else if (ordered.contains("RequestMovementType")) {
        movement = createRequestMovementType(ordered.at("RequestMovementType"));
        if (!movement) throw std::runtime_error("Invalid movement action");
    }
    if (ordered.contains("RequestSinglePose") || ordered.contains("RequestHeadOrientation")) {
        if (!movement) throw std::runtime_error("Legacy pose components require RequestMovementType");
        YAML::Node pose(YAML::NodeType::Map);
        if (ordered.contains("RequestSinglePose")) {
            auto torso = YAML::Clone(ordered.at("RequestSinglePose"));
            torso.remove("minDuration");
            pose["torso_pose"] = torso;
            ordered.erase("RequestSinglePose");
        }
        if (ordered.contains("RequestHeadOrientation")) {
            auto head = YAML::Clone(ordered.at("RequestHeadOrientation"));
            head.remove("minDuration");
            pose["head_orientation"] = head;
            ordered.erase("RequestHeadOrientation");
        }
        if (rumblex_geometry::hasBodyPose(movement->movementRequest.body_pose))
            throw std::runtime_error("Conflicting legacy and unified pose targets");
        movement->movementRequest.body_pose = parseBodyPose(pose);
    }
    if (movement && movement->movementRequest.type == MovementRequest::SINGLE_POSE &&
        !rumblex_geometry::hasBodyPose(movement->movementRequest.body_pose))
        throw std::runtime_error("SINGLE_POSE requires a body_pose target");
    if (movement &&
        (movement->movementRequest.type == MovementRequest::HOLD_DISTANCE ||
         movement->movementRequest.type == MovementRequest::LEVEL_TORSO) &&
        (ordered.contains("RequestVelocity") ||
         rumblex_geometry::hasBodyPose(movement->movementRequest.body_pose)))
        throw std::runtime_error("Sensor gaits own their velocity and pose targets");
    std::vector<std::shared_ptr<RequestBase>> requests;
    // Canonical names preserve dispatch order even when YAML aliases change.
    for (const auto& [type, value] : ordered) {
        std::shared_ptr<RequestBase> request;
        if (type != "RequestMovementType") {
            if (value.IsMap()) {
                if (type == "RequestTalking" || type == "RequestChat")
                    keys(value, {"text", "language", "minDuration"});
                else if (type == "RequestMusic")
                    keys(value, {"song", "volume", "minDuration"});
                else if (type == "RequestListening")
                    keys(value, {"active", "minDuration"});
                else if (type == "RequestSystem")
                    keys(value, {"turnOffServoRelay", "systemShutdown", "minDuration"});
                else if (type == "RequestVelocity")
                    keys(value, {"linear", "angular", "minDuration"});
            } else if (!value.IsScalar()) {
                throw std::runtime_error("Expected scalar or mapping for " + type);
            }
        }
        if (type == "RequestTalking")
            request = createRequestTalking(value);
        else if (type == "RequestChat")
            request = createRequestChat(value);
        else if (type == "RequestMusic")
            request = createRequestMusic(value);
        else if (type == "RequestListening")
            request = createRequestListening(value);
        else if (type == "RequestSystem")
            request = createRequestSystem(value);
        else if (type == "RequestMovementType")
            request = movement;
        else if (type == "RequestVelocity")
            request = createRequestMoveVelocity(value);
        else
            throw std::runtime_error("Unknown action: " + type);
        if (!request || !isfinite(request->minDuration) || request->minDuration < 0.0 * s)
            throw std::runtime_error("Invalid action: " + type);
        requests.push_back(request);
    }
    return requests;
}

std::shared_ptr<RequestTalking> CSequenceParser::createRequestTalking(const YAML::Node& value) {
    try {
        auto request = std::make_shared<RequestTalking>();

        if (value.IsScalar()) {
            request->text = value.as<std::string>();
        } else if (value.IsMap()) {
            request->text = valueOr<std::string>(value, "text", "");
            request->language = valueOr<std::string>(value, "language", "de");
            request->minDuration = valueOr(value, "minDuration", 0.0) * s;
        }

        return request;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestTalking: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestChat> CSequenceParser::createRequestChat(const YAML::Node& value) {
    try {
        auto request = std::make_shared<RequestChat>();

        if (value.IsScalar()) {
            request->text = value.as<std::string>();
        } else if (value.IsMap()) {
            request->text = valueOr<std::string>(value, "text", "");
            request->language = valueOr<std::string>(value, "language", "de");
            request->minDuration = valueOr(value, "minDuration", 0.0) * s;
        }

        return request;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestChat: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestMusic> CSequenceParser::createRequestMusic(const YAML::Node& value) {
    try {
        auto request = std::make_shared<RequestMusic>();

        if (value.IsScalar()) {
            request->song = value.as<std::string>();
        } else if (value.IsMap()) {
            request->song = valueOr<std::string>(value, "song", "");
            request->volume = valueOr(value, "volume", 0.8f);
            if (!std::isfinite(request->volume)) throw std::runtime_error("Non-finite music volume");
            request->minDuration = valueOr(value, "minDuration", 0.0) * s;
        }

        return request;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestMusic: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestListening> CSequenceParser::createRequestListening(const YAML::Node& value) {
    try {
        auto request = std::make_shared<RequestListening>();

        if (value.IsScalar()) {
            request->active = value.as<bool>();
        } else if (value.IsMap()) {
            request->active = valueOr(value, "active", false);
            request->minDuration = valueOr(value, "minDuration", 0.0) * s;
        }

        return request;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestListening: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestSystem> CSequenceParser::createRequestSystem(const YAML::Node& value) {
    try {
        if (value.IsMap()) {
            auto request = std::make_shared<RequestSystem>();
            request->turnOffServoRelay = valueOr(value, "turnOffServoRelay", false);
            request->systemShutdown = valueOr(value, "systemShutdown", false);
            request->minDuration = valueOr(value, "minDuration", 0.0) * s;
            return request;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestSystem: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestMovementType> CSequenceParser::createRequestMovementType(const YAML::Node& value) {
    try {
        if (value.IsMap()) {
            keys(value,
                 {"name", "type", "direction", "duration_s", "minDuration", "body_pose", "distance_m"});
            if (value["name"] && value["type"]) throw std::runtime_error("Duplicate movement name/type");
            brain::MovementRequest movementRequest;
            if (value["body_pose"]) {
                if (value["body_pose"]["duration_s"]) throw std::runtime_error("Put duration_s on movement");
                movementRequest.body_pose = parseBodyPose(value["body_pose"]);
            }

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
                    throw std::runtime_error("Unknown movement type: " + typeStr);
                }
            }

            if (typeStr.empty()) throw std::runtime_error("Missing movement name");
            // Parse direction if present
            if (value["direction"]) {
                std::string directionStr = value["direction"].as<std::string>();
                if (directionStr == "CLOCKWISE") {
                    movementRequest.direction = brain::MovementRequest::CLOCKWISE;
                } else if (directionStr == "ANTICLOCKWISE") {
                    movementRequest.direction = brain::MovementRequest::ANTICLOCKWISE;
                } else {
                    throw std::runtime_error("Unknown direction: " + directionStr);
                }
            }
            if (value["duration_s"]) {
                movementRequest.duration_s = value["duration_s"].as<double>() * s;
            }

            if (value["distance_m"]) movementRequest.distance_m = value["distance_m"].as<double>() * m;
            const bool sensor_gait = movementRequest.type == MovementRequest::HOLD_DISTANCE ||
                                     movementRequest.type == MovementRequest::LEVEL_TORSO;
            if (movementRequest.type == MovementRequest::HOLD_DISTANCE) {
                if (!movementRequest.distance_m || !isfinite(*movementRequest.distance_m) ||
                    *movementRequest.distance_m <= 0.0 * m)
                    throw std::runtime_error("HOLD_DISTANCE requires positive finite distance_m");
            } else if (movementRequest.distance_m) {
                throw std::runtime_error("distance_m requires HOLD_DISTANCE");
            }
            if (sensor_gait && (movementRequest.duration_s <= 0.0 * s ||
                                rumblex_geometry::hasBodyPose(movementRequest.body_pose)))
                throw std::runtime_error("Sensor gaits require positive duration and own their pose targets");
            if (!isfinite(movementRequest.duration_s) || movementRequest.duration_s < 0.0 * s)
                throw std::runtime_error("Invalid movement duration");
            if (movementRequest.type != MovementRequest::SINGLE_POSE &&
                movementRequest.body_pose.has_toe_positions)
                throw std::runtime_error("Toe targets require SINGLE_POSE");
            auto request = std::make_shared<RequestMovementType>();
            request->movementRequest = movementRequest;
            request->minDuration = valueOr(value, "minDuration", 0.0) * s;
            return request;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestMovementType: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestVelocity> CSequenceParser::createRequestMoveVelocity(const YAML::Node& value) {
    try {
        if (value.IsMap()) {
            geometry_msgs::msg::Twist velocity;

            if (value["linear"]) parseVector(value["linear"], velocity.linear);
            if (value["angular"]) parseVector(value["angular"], velocity.angular);
            for (double v : {velocity.linear.x, velocity.linear.y, velocity.linear.z, velocity.angular.x,
                             velocity.angular.y, velocity.angular.z})
                if (!std::isfinite(v)) throw std::runtime_error("Non-finite velocity");

            auto request = std::make_shared<RequestVelocity>();
            request->velocity = velocity;
            request->minDuration = valueOr(value, "minDuration", 0.0) * s;
            return request;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestVelocity: %s", e.what());
    }

    return nullptr;
}

}  // namespace brain
