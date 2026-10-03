/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "requester/action_packages_parser.hpp"

#include <yaml-cpp/yaml.h>

namespace rumblex_movement {

CActionPackagesParser::CActionPackagesParser(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    readYaml();
}

void CActionPackagesParser::readYaml() {
    const auto package_share_path = ament_index_cpp::get_package_share_path("rumblex_movement");
    const auto yaml_path = (package_share_path / "config" / "actionpackages.yaml").string();

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
                // each step should be a map describing head/torso/legs/toe_positions
                if (!step.IsMap()) {
                    RCLCPP_WARN_STREAM(node_->get_logger(),
                                       "Skipping invalid step (not a map) in package: " << key);
                    continue;
                }
                parseYamlStep(step, action_package);
            }
            action_packages_[key] = action_package;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Error parsing YAML: " << yaml_path << " : " << e.what());
        return;
    }

    RCLCPP_INFO_STREAM(node_->get_logger(), "Loaded default values for presets.");
    RCLCPP_INFO_STREAM(node_->get_logger(), "Default LegAngles:");
    for (const auto& [key, val] : default_leg_angles_) {
        RCLCPP_INFO_STREAM(node_->get_logger(), " - " << key);
    }
    RCLCPP_INFO_STREAM(node_->get_logger(), "Default ToePositions:");
    for (const auto& [key, val] : default_toe_positions_) {
        RCLCPP_INFO_STREAM(node_->get_logger(), " - " << key);
    }
    RCLCPP_INFO_STREAM(node_->get_logger(), "Default Heads:");
    for (const auto& [key, val] : default_heads_) {
        RCLCPP_INFO_STREAM(node_->get_logger(), " - " << key);
    }
    RCLCPP_INFO_STREAM(node_->get_logger(), "Default Torsos:");
    for (const auto& [key, val] : default_torsos_) {
        RCLCPP_INFO_STREAM(node_->get_logger(), " - " << key);
    }

    RCLCPP_INFO_STREAM(node_->get_logger(), "Loaded " << action_packages_.size() << " action packages.");
    RCLCPP_INFO_STREAM(node_->get_logger(), "keys:");
    for (const auto& [key, _] : action_packages_) {
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

            if (key.rfind("leg_angles", 0) == 0) {
                parsePresetLegAngles(key, val);
                continue;
            }
            if (key.rfind("toe_positions", 0) == 0) {
                parsePresetToePositions(key, val);
                continue;
            }
            if (key.rfind("head", 0) == 0) {
                parsePresetHead(key, val);
                continue;
            }
            if (key.rfind("torso", 0) == 0) {
                parsePresetTorso(key, val);
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
    const YAML::Node all_node = val["all"];
    if (all_node && all_node.IsMap()) {
        const double torso_coxa = all_node["torso_coxa"] ? all_node["torso_coxa"].as<double>() : 0.0;
        const double coxa_femur = all_node["coxa_femur"] ? all_node["coxa_femur"].as<double>() : 0.0;
        const double femur_tibia = all_node["femur_tibia"] ? all_node["femur_tibia"].as<double>() : 0.0;
        for (ELegIndex idx : magic_enum::enum_values<ELegIndex>()) {
            default_leg_angles_[key][idx] = CLegAngles(torso_coxa, coxa_femur, femur_tibia);
        }
    }
    // Per-leg overrides
    for (auto it2 = val.begin(); it2 != val.end(); ++it2) {
        const std::string leg_name = it2->first.as<std::string>();
        if (leg_name == "all") continue;
        const YAML::Node& node = it2->second;
        if (!node || !node.IsMap()) continue;
        auto idx_opt = parseLegIndex(leg_name);
        if (!idx_opt.has_value()) {
            RCLCPP_DEBUG_STREAM(node_->get_logger(), "Unknown leg key in leg_angles preset: " << leg_name);
            continue;
        }
        const double torso_coxa = node["torso_coxa"] ? node["torso_coxa"].as<double>() : 0.0;
        const double coxa_femur = node["coxa_femur"] ? node["coxa_femur"].as<double>() : 0.0;
        const double femur_tibia = node["femur_tibia"] ? node["femur_tibia"].as<double>() : 0.0;
        default_leg_angles_[key][*idx_opt] = CLegAngles(torso_coxa, coxa_femur, femur_tibia);
    }
}

void CActionPackagesParser::parsePresetToePositions(const std::string& key, const YAML::Node& val) {
    if (!val || !val.IsMap()) return;
    // Apply 'All' first
    const YAML::Node all_node = val["all"];
    if (all_node && all_node.IsMap()) {
        const double x = all_node["x"] ? all_node["x"].as<double>() : 0.0;
        const double y = all_node["y"] ? all_node["y"].as<double>() : 0.0;
        const double z = all_node["z"] ? all_node["z"].as<double>() : 0.0;
        for (ELegIndex idx : magic_enum::enum_values<ELegIndex>()) {
            default_toe_positions_[key][idx] = CPosition(x, y, z);
        }
    }
    // Per-leg overrides
    for (auto it2 = val.begin(); it2 != val.end(); ++it2) {
        const std::string leg_name = it2->first.as<std::string>();
        if (leg_name == "all") continue;
        const YAML::Node& node = it2->second;
        if (!node || !node.IsMap()) continue;
        auto idx_opt = parseLegIndex(leg_name);
        if (!idx_opt.has_value()) {
            RCLCPP_DEBUG_STREAM(node_->get_logger(), "Unknown leg key in toePositions preset: " << leg_name);
            continue;
        }
        const double x = node["x"] ? node["x"].as<double>() : 0.0;
        const double y = node["y"] ? node["y"].as<double>() : 0.0;
        const double z = node["z"] ? node["z"].as<double>() : 0.0;
        default_toe_positions_[key][*idx_opt] = CPosition(x, y, z);
    }
}

void CActionPackagesParser::parsePresetHead(const std::string& key, const YAML::Node& val) {
    const double yaw = val["yaw"] ? val["yaw"].as<double>() : 0.0;
    const double pitch = val["pitch"] ? val["pitch"].as<double>() : 0.0;
    default_heads_[key] = COrientation(0.0, pitch, yaw);
}

void CActionPackagesParser::parsePresetTorso(const std::string& key, const YAML::Node& val) {
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
    default_torsos_[key] = CPose(x, y, z, roll_deg, pitch_deg, yaw_deg);
}

void CActionPackagesParser::parseYamlStep(const YAML::Node& step,
                                          std::vector<CActionPackage>& action_package) {
    CActionPackage action;

    // No debug dumps: keep output quiet in CI

    if (step["duration_factor"]) {
        action.duration_factor = step["duration_factor"].as<double>();
    } else {
        RCLCPP_ERROR_STREAM(
            node_->get_logger(),
            "CActionPackagesParser::parseYamlStep: 'duration_factor' not found, defaulting to 1.0");
        action.duration_factor = 1.0;
    }

    if (step["head"]) {
        action.head = parseHeadNode(step["head"]);
    }

    if (step["torso"]) {
        action.torso = parseTorsoNode(step["torso"]);
    }

    if (step["leg_angles"]) {
        auto legs_map = parseLegAnglesNode(step["leg_angles"]);
        if (!legs_map.empty()) action.leg_angles = legs_map;
    }

    if (step["toe_positions"]) {
        auto pos_map = parseToePositionsNode(step["toe_positions"]);
        if (!pos_map.empty()) action.toe_positions = pos_map;
    }

    action_package.push_back(action);
}

const std::vector<CActionPackage>& CActionPackagesParser::getRequests(const std::string& package_name) {
    if (action_packages_.find(package_name) != action_packages_.end()) {
        auto& vec = action_packages_.at(package_name);
        // No diagnostic logging here to keep test output clean
        return vec;
    } else {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Action package not found: " << package_name);
        static const std::vector<CActionPackage> empty_vector;
        return empty_vector;
    }
}

std::map<ELegIndex, CPosition> CActionPackagesParser::getToePositions(const std::string& name) {
    if (default_toe_positions_.find(name) != default_toe_positions_.end()) {
        return default_toe_positions_.at(name);
    } else {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Default toe positions not found: " << name);
        return std::map<ELegIndex, CPosition>();
    }
}

std::map<ELegIndex, CLegAngles> CActionPackagesParser::getLegAngles(const std::string& name) {
    if (default_leg_angles_.find(name) != default_leg_angles_.end()) {
        return default_leg_angles_.at(name);
    } else {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Default leg angles not found: " << name);
        return std::map<ELegIndex, CLegAngles>();
    }
}

COrientation CActionPackagesParser::getHeadOrientation(const std::string& name) {
    if (default_heads_.find(name) != default_heads_.end()) {
        return default_heads_.at(name);
    } else {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Default head not found: " << name);
        return COrientation();
    }
}
CPose CActionPackagesParser::getTorsoPose(const std::string& name) {
    if (default_torsos_.find(name) != default_torsos_.end()) {
        return default_torsos_.at(name);
    } else {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Default torso not found: " << name);
        return CPose();
    }
}

COrientation CActionPackagesParser::parseHeadNode(const YAML::Node& head_node) {
    double yaw = 0.0, pitch = 0.0;
    // head can be a sequence of maps or a single map
    if (head_node.IsSequence()) {
        for (const auto& entry : head_node) {
            if (entry["yaw"]) yaw = entry["yaw"].as<double>();
            if (entry["pitch"]) pitch = entry["pitch"].as<double>();
        }
    } else if (head_node.IsMap()) {
        if (head_node["yaw"]) yaw = head_node["yaw"].as<double>();
        if (head_node["pitch"]) pitch = head_node["pitch"].as<double>();
    }
    return COrientation(0.0, pitch, yaw);
}

CPose CActionPackagesParser::parseTorsoNode(const YAML::Node& torso_node) {
    double roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;
    double x = 0.0, y = 0.0, z = 0.0;

    std::vector<YAML::Node> torso_entries;
    if (torso_node.IsSequence()) {
        for (const auto& n : torso_node) torso_entries.push_back(n);
    } else if (torso_node.IsMap()) {
        torso_entries.push_back(torso_node);
    }

    for (const auto& entry : torso_entries) {
        if (entry["orientation"]) {
            YAML::Node orient_node = entry["orientation"];
            if (orient_node.IsSequence()) {
                for (const auto& orientation_entry : orient_node) {
                    if (orientation_entry["roll"]) roll_deg = orientation_entry["roll"].as<double>();
                    if (orientation_entry["pitch"]) pitch_deg = orientation_entry["pitch"].as<double>();
                    if (orientation_entry["yaw"]) yaw_deg = orientation_entry["yaw"].as<double>();
                }
            } else if (orient_node.IsMap()) {
                if (orient_node["roll"]) roll_deg = orient_node["roll"].as<double>();
                if (orient_node["pitch"]) pitch_deg = orient_node["pitch"].as<double>();
                if (orient_node["yaw"]) yaw_deg = orient_node["yaw"].as<double>();
            }
        }
        if (entry["direction"]) {
            YAML::Node dir_node = entry["direction"];
            if (dir_node.IsSequence()) {
                for (const auto& direction_entry : dir_node) {
                    if (direction_entry["x"]) x = direction_entry["x"].as<double>();
                    if (direction_entry["y"]) y = direction_entry["y"].as<double>();
                    if (direction_entry["z"]) z = direction_entry["z"].as<double>();
                }
            } else if (dir_node.IsMap()) {
                if (dir_node["x"]) x = dir_node["x"].as<double>();
                if (dir_node["y"]) y = dir_node["y"].as<double>();
                if (dir_node["z"]) z = dir_node["z"].as<double>();
            }
        }
    }

    return CPose(x, y, z, roll_deg, pitch_deg, yaw_deg);
}

std::map<ELegIndex, CLegAngles> CActionPackagesParser::parseLegAnglesNode(const YAML::Node& legs_node) {
    std::map<ELegIndex, CLegAngles> legs_map;
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
                if (key == "all") {
                    double torso_coxa = val["torso_coxa"] ? val["torso_coxa"].as<double>() : 0.0;
                    double coxa_femur = val["coxa_femur"] ? val["coxa_femur"].as<double>() : 0.0;
                    double femur_tibia = val["femur_tibia"] ? val["femur_tibia"].as<double>() : 0.0;
                    for (ELegIndex idx : magic_enum::enum_values<ELegIndex>()) {
                        legs_map[idx] = CLegAngles(torso_coxa, coxa_femur, femur_tibia);
                    }
                } else if (auto idx_opt = parseLegIndex(key); idx_opt.has_value()) {
                    double torso_coxa = val["torso_coxa"] ? val["torso_coxa"].as<double>() : 0.0;
                    double coxa_femur = val["coxa_femur"] ? val["coxa_femur"].as<double>() : 0.0;
                    double femur_tibia = val["femur_tibia"] ? val["femur_tibia"].as<double>() : 0.0;
                    legs_map[*idx_opt] = CLegAngles(torso_coxa, coxa_femur, femur_tibia);
                } else {
                    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Unknown leg key in legs map: " << key);
                }
            }
            return;
        }
    };
    process(legs_node);
    return legs_map;
}

std::map<ELegIndex, CPosition> CActionPackagesParser::parseToePositionsNode(const YAML::Node& pos_node) {
    std::map<ELegIndex, CPosition> pos_map;
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
                if (key == "all") {
                    double x = val["x"] ? val["x"].as<double>() : 0.0;
                    double y = val["y"] ? val["y"].as<double>() : 0.0;
                    double z = val["z"] ? val["z"].as<double>() : 0.0;
                    for (ELegIndex idx : magic_enum::enum_values<ELegIndex>())
                        pos_map[idx] = CPosition(x, y, z);
                } else if (auto idx_opt = parseLegIndex(key); idx_opt.has_value()) {
                    double x = val["x"] ? val["x"].as<double>() : 0.0;
                    double y = val["y"] ? val["y"].as<double>() : 0.0;
                    double z = val["z"] ? val["z"].as<double>() : 0.0;
                    pos_map[*idx_opt] = CPosition(x, y, z);
                } else {
                    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Unknown leg key in toePositions: " << key);
                }
            }
            return;
        }
    };
    process(pos_node);
    return pos_map;
}

}  // namespace rumblex_movement