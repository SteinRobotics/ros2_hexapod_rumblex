/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "requester/actionpackagesparser.hpp"

#include <yaml-cpp/yaml.h>

namespace rumblex_movement {

CActionPackagesParser::CActionPackagesParser(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    readYaml();
}

void CActionPackagesParser::readYaml() {
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("rumblex_movement");
    std::string yaml_path = package_share_directory + "/config/actionpackages.yaml";

    try {
        YAML::Node yaml_data = YAML::LoadFile(yaml_path);

        for (const auto& it : yaml_data) {
            std::string key = it.first.as<std::string>();

            RCLCPP_INFO_STREAM(node_->get_logger(), "Processing top-level key: " << key);

            if (key == "presets") {
                parseDefaultValues(it.second);
                continue;  // move to next top-level key
            }

            if (!it.second.IsSequence()) {
                RCLCPP_DEBUG_STREAM(node_->get_logger(), "Skipping non-sequence top-level key: " << key);
                continue;
            }
            std::vector<CActionPackage> action_package;
            for (const auto& step : it.second) {
                // each step should be a map describing head/body/legs/footPositions
                if (!step.IsMap()) {
                    RCLCPP_WARN_STREAM(node_->get_logger(),
                                       "Skipping invalid step (not a map) in package: " << key);
                    continue;
                }
                parseYamlStep(step, action_package);
            }
            actionPackages_[key] = action_package;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Error parsing YAML: " << yaml_path << " : " << e.what());
        return;
    }

    RCLCPP_INFO_STREAM(node_->get_logger(), "Loaded default values for presets.");
    RCLCPP_INFO_STREAM(node_->get_logger(), "Default LegAngles:");
    for (const auto& [key, val] : defaultLegAngles_) {
        RCLCPP_INFO_STREAM(node_->get_logger(), " - " << key);
    }
    RCLCPP_INFO_STREAM(node_->get_logger(), "Default FootPositions:");
    for (const auto& [key, val] : defaultFootPositions_) {
        RCLCPP_INFO_STREAM(node_->get_logger(), " - " << key);
    }
    RCLCPP_INFO_STREAM(node_->get_logger(), "Default Heads:");
    for (const auto& [key, val] : defaultHeads_) {
        RCLCPP_INFO_STREAM(node_->get_logger(), " - " << key);
    }
    RCLCPP_INFO_STREAM(node_->get_logger(), "Default Bodies:");
    for (const auto& [key, val] : defaultBodies_) {
        RCLCPP_INFO_STREAM(node_->get_logger(), " - " << key);
    }

    RCLCPP_INFO_STREAM(node_->get_logger(), "Loaded " << actionPackages_.size() << " action packages.");
    RCLCPP_INFO_STREAM(node_->get_logger(), "keys:");
    for (const auto& [key, _] : actionPackages_) {
        RCLCPP_INFO_STREAM(node_->get_logger(), " - " << key);
    }
}

void CActionPackagesParser::parseDefaultValues(const YAML::Node& defaults) {
    if (!defaults || !defaults.IsMap()) {
        RCLCPP_WARN(node_->get_logger(), "parseDefaultValues: 'presets' node is missing or not a map");
        return;
    }

    try {
        for (const auto& it : defaults) {
            std::string key = it.first.as<std::string>();
            const YAML::Node& val = it.second;
            RCLCPP_INFO_STREAM(node_->get_logger(), "Parsing default values for key: " << key);

            if (key.rfind("legAngles", 0) == 0) {
                parsePresetLegAngles(key, val);
                continue;
            }
            if (key.rfind("footPositions", 0) == 0) {
                parsePresetFootPositions(key, val);
                continue;
            }
            if (key.rfind("head", 0) == 0) {
                parsePresetHead(key, val);
                continue;
            }
            if (key.rfind("body", 0) == 0) {
                parsePresetBody(key, val);
                continue;
            }
        }
    } catch (const YAML::Exception& ex) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "parseDefaultValues: YAML exception: " << ex.what());
    }
}

void CActionPackagesParser::parsePresetLegAngles(const std::string& key, const YAML::Node& val) {
    if (!val || !val.IsMap()) return;
    // Apply 'All' first
    const YAML::Node allNode = val["All"];
    if (allNode && allNode.IsMap()) {
        const double coxa = allNode["coxa"] ? allNode["coxa"].as<double>() : 0.0;
        const double femur = allNode["femur"] ? allNode["femur"].as<double>() : 0.0;
        const double tibia = allNode["tibia"] ? allNode["tibia"].as<double>() : 0.0;
        for (ELegIndex idx : magic_enum::enum_values<ELegIndex>()) {
            defaultLegAngles_[key][idx] = CLegAngles(coxa, femur, tibia);
        }
    }
    // Per-leg overrides
    for (auto it2 = val.begin(); it2 != val.end(); ++it2) {
        const std::string leg_name = it2->first.as<std::string>();
        if (leg_name == "All") continue;
        const YAML::Node& node = it2->second;
        if (!node || !node.IsMap()) continue;
        auto idxOpt = magic_enum::enum_cast<ELegIndex>(leg_name);
        if (!idxOpt.has_value()) {
            RCLCPP_DEBUG_STREAM(node_->get_logger(), "Unknown leg key in legAngles preset: " << leg_name);
            continue;
        }
        const double coxa = node["coxa"] ? node["coxa"].as<double>() : 0.0;
        const double femur = node["femur"] ? node["femur"].as<double>() : 0.0;
        const double tibia = node["tibia"] ? node["tibia"].as<double>() : 0.0;
        defaultLegAngles_[key][*idxOpt] = CLegAngles(coxa, femur, tibia);
    }
}

void CActionPackagesParser::parsePresetFootPositions(const std::string& key, const YAML::Node& val) {
    if (!val || !val.IsMap()) return;
    // Apply 'All' first
    const YAML::Node allNode = val["All"];
    if (allNode && allNode.IsMap()) {
        const double x = allNode["x"] ? allNode["x"].as<double>() : 0.0;
        const double y = allNode["y"] ? allNode["y"].as<double>() : 0.0;
        const double z = allNode["z"] ? allNode["z"].as<double>() : 0.0;
        for (ELegIndex idx : magic_enum::enum_values<ELegIndex>()) {
            defaultFootPositions_[key][idx] = CPosition(x, y, z);
        }
    }
    // Per-leg overrides
    for (auto it2 = val.begin(); it2 != val.end(); ++it2) {
        const std::string leg_name = it2->first.as<std::string>();
        if (leg_name == "All") continue;
        const YAML::Node& node = it2->second;
        if (!node || !node.IsMap()) continue;
        auto idxOpt = magic_enum::enum_cast<ELegIndex>(leg_name);
        if (!idxOpt.has_value()) {
            RCLCPP_DEBUG_STREAM(node_->get_logger(), "Unknown leg key in footPositions preset: " << leg_name);
            continue;
        }
        const double x = node["x"] ? node["x"].as<double>() : 0.0;
        const double y = node["y"] ? node["y"].as<double>() : 0.0;
        const double z = node["z"] ? node["z"].as<double>() : 0.0;
        defaultFootPositions_[key][*idxOpt] = CPosition(x, y, z);
    }
}

void CActionPackagesParser::parsePresetHead(const std::string& key, const YAML::Node& val) {
    const double yaw = val["yaw"] ? val["yaw"].as<double>() : 0.0;
    const double pitch = val["pitch"] ? val["pitch"].as<double>() : 0.0;
    defaultHeads_[key] = COrientation(0.0, pitch, yaw);
}

void CActionPackagesParser::parsePresetBody(const std::string& key, const YAML::Node& val) {
    double roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;
    double x = 0.0, y = 0.0, z = 0.0;
    if (val["orientation"] && val["orientation"].IsMap()) {
        const auto& o = val["orientation"];
        roll_deg = o["roll"] ? o["roll"].as<double>() : 0.0;
        pitch_deg = o["pitch"] ? o["pitch"].as<double>() : 0.0;
        yaw_deg = o["yaw"] ? o["yaw"].as<double>() : 0.0;
    }
    if (val["direction"] && val["direction"].IsMap()) {
        const auto& d = val["direction"];
        x = d["x"] ? d["x"].as<double>() : 0.0;
        y = d["y"] ? d["y"].as<double>() : 0.0;
        z = d["z"] ? d["z"].as<double>() : 0.0;
    }
    defaultBodies_[key] = CPose(x, y, z, roll_deg, pitch_deg, yaw_deg);
}

void CActionPackagesParser::parseYamlStep(const YAML::Node& step,
                                          std::vector<CActionPackage>& action_package) {
    CActionPackage action;

    // No debug dumps: keep output quiet in CI

    if (step["factorDuration"]) {
        action.factorDuration = step["factorDuration"].as<double>();
    } else {
        RCLCPP_ERROR_STREAM(
            node_->get_logger(),
            "CActionPackagesParser::parseYamlStep: 'factorDuration' not found, defaulting to 1.0");
        action.factorDuration = 1.0;
    }

    if (step["head"]) {
        action.head = parseHeadNode(step["head"]);
    }

    if (step["body"]) {
        action.body = parseBodyNode(step["body"]);
    }

    if (step["legAngles"]) {
        auto legsMap = parseLegAnglesNode(step["legAngles"]);
        if (!legsMap.empty()) action.legAngles = legsMap;
    }

    if (step["footPositions"]) {
        auto posMap = parseFootPositionsNode(step["footPositions"]);
        if (!posMap.empty()) action.footPositions = posMap;
    }

    action_package.push_back(action);
}

const std::vector<CActionPackage>& CActionPackagesParser::getRequests(const std::string& packageName) {
    if (actionPackages_.find(packageName) != actionPackages_.end()) {
        auto& vec = actionPackages_.at(packageName);
        // No diagnostic logging here to keep test output clean
        return vec;
    } else {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Action package not found: " << packageName);
        static const std::vector<CActionPackage> empty_vector;
        return empty_vector;
    }
}

std::map<ELegIndex, CPosition> CActionPackagesParser::getFootPositions(const std::string& name) {
    if (defaultFootPositions_.find(name) != defaultFootPositions_.end()) {
        return defaultFootPositions_.at(name);
    } else {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Default foot positions not found: " << name);
        return std::map<ELegIndex, CPosition>();
    }
}

std::map<ELegIndex, CLegAngles> CActionPackagesParser::getLegAngles(const std::string& name) {
    if (defaultLegAngles_.find(name) != defaultLegAngles_.end()) {
        return defaultLegAngles_.at(name);
    } else {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Default leg angles not found: " << name);
        return std::map<ELegIndex, CLegAngles>();
    }
}

COrientation CActionPackagesParser::getHead(const std::string& name) {
    if (defaultHeads_.find(name) != defaultHeads_.end()) {
        return defaultHeads_.at(name);
    } else {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Default head not found: " << name);
        return COrientation();
    }
}
CPose CActionPackagesParser::getBody(const std::string& name) {
    if (defaultBodies_.find(name) != defaultBodies_.end()) {
        return defaultBodies_.at(name);
    } else {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Default body not found: " << name);
        return CPose();
    }
}

COrientation CActionPackagesParser::parseHeadNode(const YAML::Node& headNode) {
    double yaw = 0.0, pitch = 0.0;
    // head can be a sequence of maps or a single map
    if (headNode.IsSequence()) {
        for (const auto& entry : headNode) {
            if (entry["yaw"]) yaw = entry["yaw"].as<double>();
            if (entry["pitch"]) pitch = entry["pitch"].as<double>();
        }
    } else if (headNode.IsMap()) {
        if (headNode["yaw"]) yaw = headNode["yaw"].as<double>();
        if (headNode["pitch"]) pitch = headNode["pitch"].as<double>();
    }
    return COrientation(0.0, pitch, yaw);
}

CPose CActionPackagesParser::parseBodyNode(const YAML::Node& bodyNode) {
    double roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;
    double x = 0.0, y = 0.0, z = 0.0;

    std::vector<YAML::Node> bodyEntries;
    if (bodyNode.IsSequence()) {
        for (const auto& n : bodyNode) bodyEntries.push_back(n);
    } else if (bodyNode.IsMap()) {
        bodyEntries.push_back(bodyNode);
    }

    for (const auto& entry : bodyEntries) {
        if (entry["orientation"]) {
            YAML::Node orientNode = entry["orientation"];
            if (orientNode.IsSequence()) {
                for (const auto& orientationEntry : orientNode) {
                    if (orientationEntry["roll"]) roll_deg = orientationEntry["roll"].as<double>();
                    if (orientationEntry["pitch"]) pitch_deg = orientationEntry["pitch"].as<double>();
                    if (orientationEntry["yaw"]) yaw_deg = orientationEntry["yaw"].as<double>();
                }
            } else if (orientNode.IsMap()) {
                if (orientNode["roll"]) roll_deg = orientNode["roll"].as<double>();
                if (orientNode["pitch"]) pitch_deg = orientNode["pitch"].as<double>();
                if (orientNode["yaw"]) yaw_deg = orientNode["yaw"].as<double>();
            }
        }
        if (entry["direction"]) {
            YAML::Node dirNode = entry["direction"];
            if (dirNode.IsSequence()) {
                for (const auto& directionEntry : dirNode) {
                    if (directionEntry["x"]) x = directionEntry["x"].as<double>();
                    if (directionEntry["y"]) y = directionEntry["y"].as<double>();
                    if (directionEntry["z"]) z = directionEntry["z"].as<double>();
                }
            } else if (dirNode.IsMap()) {
                if (dirNode["x"]) x = dirNode["x"].as<double>();
                if (dirNode["y"]) y = dirNode["y"].as<double>();
                if (dirNode["z"]) z = dirNode["z"].as<double>();
            }
        }
    }

    return CPose(x, y, z, roll_deg, pitch_deg, yaw_deg);
}

std::map<ELegIndex, CLegAngles> CActionPackagesParser::parseLegAnglesNode(const YAML::Node& legsNode) {
    std::map<ELegIndex, CLegAngles> legsMap;
    std::function<void(const YAML::Node&)> process;
    process = [&](const YAML::Node& node) {
        if (node.IsNull()) return;
        if (node.IsSequence()) {
            for (const auto& elem : node) process(elem);
            return;
        }
        if (node.IsMap()) {
            for (const auto& kv : node) {
                std::string key = kv.first.as<std::string>();
                const YAML::Node& val = kv.second;
                if (key == "<<") {
                    process(val);
                    continue;
                }
                if (key == "All") {
                    double coxa = val["coxa"] ? val["coxa"].as<double>() : 0.0;
                    double femur = val["femur"] ? val["femur"].as<double>() : 0.0;
                    double tibia = val["tibia"] ? val["tibia"].as<double>() : 0.0;
                    for (ELegIndex idx : magic_enum::enum_values<ELegIndex>()) {
                        legsMap[idx] = CLegAngles(coxa, femur, tibia);
                    }
                } else if (auto idxOpt = magic_enum::enum_cast<ELegIndex>(key); idxOpt.has_value()) {
                    double coxa = val["coxa"] ? val["coxa"].as<double>() : 0.0;
                    double femur = val["femur"] ? val["femur"].as<double>() : 0.0;
                    double tibia = val["tibia"] ? val["tibia"].as<double>() : 0.0;
                    legsMap[*idxOpt] = CLegAngles(coxa, femur, tibia);
                } else {
                    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Unknown leg key in legs map: " << key);
                }
            }
            return;
        }
    };
    process(legsNode);
    return legsMap;
}

std::map<ELegIndex, CPosition> CActionPackagesParser::parseFootPositionsNode(const YAML::Node& posNode) {
    std::map<ELegIndex, CPosition> posMap;
    std::function<void(const YAML::Node&)> process;
    process = [&](const YAML::Node& node) {
        if (node.IsNull()) return;
        if (node.IsSequence()) {
            for (const auto& elem : node) process(elem);
            return;
        }
        if (node.IsMap()) {
            for (const auto& kv : node) {
                std::string key = kv.first.as<std::string>();
                const YAML::Node& val = kv.second;
                if (key == "<<") {
                    process(val);
                    continue;
                }
                if (key == "All") {
                    double x = val["x"] ? val["x"].as<double>() : 0.0;
                    double y = val["y"] ? val["y"].as<double>() : 0.0;
                    double z = val["z"] ? val["z"].as<double>() : 0.0;
                    for (ELegIndex idx : magic_enum::enum_values<ELegIndex>())
                        posMap[idx] = CPosition(x, y, z);
                } else if (auto idxOpt = magic_enum::enum_cast<ELegIndex>(key); idxOpt.has_value()) {
                    double x = val["x"] ? val["x"].as<double>() : 0.0;
                    double y = val["y"] ? val["y"].as<double>() : 0.0;
                    double z = val["z"] ? val["z"].as<double>() : 0.0;
                    posMap[*idxOpt] = CPosition(x, y, z);
                } else {
                    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Unknown leg key in footPositions: " << key);
                }
            }
            return;
        }
    };
    process(posNode);
    return posMap;
}

}  // namespace rumblex_movement