/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
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
    std::optional<std::map<ELegIndex, CLegAngles>> legAngles;
    std::optional<std::map<ELegIndex, CPosition>> toePositions;
    double factorDuration = 1.0;
};

class CActionPackagesParser {
   public:
    CActionPackagesParser(std::shared_ptr<rclcpp::Node> node);
    virtual ~CActionPackagesParser() = default;

    const std::vector<CActionPackage>& getRequests(const std::string& packageName);
    std::map<ELegIndex, CPosition> getToePositions(const std::string& name);
    std::map<ELegIndex, CLegAngles> getLegAngles(const std::string& name);
    COrientation getHead(const std::string& name);
    CPose getTorso(const std::string& name);

   private:
    void readYaml();
    void parseYamlStep(const YAML::Node& step, std::vector<CActionPackage>& actionPackage);
    void parseDefaultValues(const YAML::Node& defaults);
    COrientation parseHeadNode(const YAML::Node& headNode);
    CPose parseTorsoNode(const YAML::Node& torsoNode);
    std::map<ELegIndex, CLegAngles> parseLegAnglesNode(const YAML::Node& legsNode);
    std::map<ELegIndex, CPosition> parseToePositionsNode(const YAML::Node& posNode);
    // Preset (top-level 'presets') helpers
    void parsePresetLegAngles(const std::string& key, const YAML::Node& val);
    void parsePresetToePositions(const std::string& key, const YAML::Node& val);
    void parsePresetHead(const std::string& key, const YAML::Node& val);
    void parsePresetTorso(const std::string& key, const YAML::Node& val);

    std::shared_ptr<rclcpp::Node> node_;
    std::unordered_map<std::string, std::vector<CActionPackage>> actionPackages_;
    std::unordered_map<std::string, std::map<ELegIndex, CLegAngles>> defaultLegAngles_;
    std::unordered_map<std::string, std::map<ELegIndex, CPosition>> defaultToePositions_;
    std::unordered_map<std::string, COrientation> defaultHeads_;
    std::unordered_map<std::string, CPose> defaultTorsos_;
};
}  // namespace rumblex_movement