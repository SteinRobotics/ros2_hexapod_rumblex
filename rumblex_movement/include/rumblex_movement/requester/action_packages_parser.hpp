/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_path.hpp>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"

// Include POD type definitions (ELegIndex, COrientation, CPose, CLegAngles, CPosition)
#include "requester/types.hpp"

namespace rumblex_movement {

class CActionPackage {
   public:
    std::optional<COrientation> head;
    std::optional<CPose> torso;
    std::optional<std::map<ELegIndex, CLegAngles>> leg_angles;
    std::optional<std::map<ELegIndex, CPosition>> toe_positions;
    double duration_factor = 1.0;
};

class CActionPackagesParser {
   public:
    CActionPackagesParser(std::shared_ptr<rclcpp::Node> node);
    virtual ~CActionPackagesParser() = default;

    const std::vector<CActionPackage>& getRequests(const std::string& package_name);
    std::map<ELegIndex, CPosition> getToePositions(const std::string& name);
    std::map<ELegIndex, CLegAngles> getLegAngles(const std::string& name);
    COrientation getHeadOrientation(const std::string& name);
    CPose getTorsoPose(const std::string& name);

   private:
    void readYaml();
    void parseYamlStep(const YAML::Node& step, std::vector<CActionPackage>& action_package);
    void parseDefaultValues(const YAML::Node& defaults);
    COrientation parseHeadNode(const YAML::Node& head_node);
    CPose parseTorsoNode(const YAML::Node& torso_node);
    std::map<ELegIndex, CLegAngles> parseLegAnglesNode(const YAML::Node& legs_node);
    std::map<ELegIndex, CPosition> parseToePositionsNode(const YAML::Node& pos_node);
    // Preset (top-level 'presets') helpers
    void parsePresetLegAngles(const std::string& key, const YAML::Node& val);
    void parsePresetToePositions(const std::string& key, const YAML::Node& val);
    void parsePresetHead(const std::string& key, const YAML::Node& val);
    void parsePresetTorso(const std::string& key, const YAML::Node& val);

    std::shared_ptr<rclcpp::Node> node_;
    std::unordered_map<std::string, std::vector<CActionPackage>> action_packages_;
    std::unordered_map<std::string, std::map<ELegIndex, CLegAngles>> default_leg_angles_;
    std::unordered_map<std::string, std::map<ELegIndex, CPosition>> default_toe_positions_;
    std::unordered_map<std::string, COrientation> default_heads_;
    std::unordered_map<std::string, CPose> default_torsos_;
};
}  // namespace rumblex_movement