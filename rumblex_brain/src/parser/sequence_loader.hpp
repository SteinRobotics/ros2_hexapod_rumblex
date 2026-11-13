#pragma once
#include <yaml-cpp/yaml.h>

#include <filesystem>
namespace brain::sequence_detail {
YAML::Node loadSequences(const YAML::Node& document, const std::filesystem::path& source, size_t& groups);
YAML::Node loadSequences(const std::filesystem::path& file, size_t& groups);
}  // namespace brain::sequence_detail
