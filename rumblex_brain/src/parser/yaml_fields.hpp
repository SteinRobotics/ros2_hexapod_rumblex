#pragma once
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <set>
#include <stdexcept>
namespace brain::sequence_detail {
// Defaults apply only to omitted fields; malformed explicit values remain errors.
template <typename T>
T valueOr(const YAML::Node& node, const char* key, const T& fallback) {
    return node[key] ? node[key].as<T>() : fallback;
}
inline void keys(const YAML::Node& node, std::initializer_list<std::string> allowed) {
    if (!node.IsMap()) throw std::runtime_error("Expected mapping");
    std::set<std::string> seen;
    for (const auto& entry : node) {
        const auto key = entry.first.as<std::string>();
        if (!seen.insert(key).second || std::find(allowed.begin(), allowed.end(), key) == allowed.end())
            throw std::runtime_error("Unknown or duplicate field: " + key);
    }
}
}  // namespace brain::sequence_detail
