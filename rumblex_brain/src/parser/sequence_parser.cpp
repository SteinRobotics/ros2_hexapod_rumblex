#include "rumblex_brain/parser/sequence_parser.hpp"

#include "action_decoder.hpp"
#include "sequence_loader.hpp"
#include "yaml_fields.hpp"
namespace brain {
using sequence_detail::keys;
using sequence_detail::loadSequences;
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

    keys(j, {"name", "trigger", "variables", "imports", "actions"});
    if (j["variables"] && !j["variables"].IsMap())
        throw std::runtime_error("Sequence variables must be a mapping");
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
                requests = sequence_detail::ActionDecoder(node_).parseActionGroup(actionNode);
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

}  // namespace brain
