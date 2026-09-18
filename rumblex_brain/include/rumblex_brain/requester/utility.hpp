/*******************************************************************************
 * Copyright (c) 2025 Christian Stein
 ******************************************************************************/

#pragma once

#include <algorithm>
#include <format>
#include <map>
#include <string>

#include "rumblex_interfaces/msg/movement_request.hpp"

namespace brain {

const std::map<const uint32_t, const std::string> movementTypeToName = {
    {rumblex_interfaces::msg::MovementRequest::NO_REQUEST, "NO_REQUEST"},
    {rumblex_interfaces::msg::MovementRequest::SEQUENCE_LAYDOWN, "SEQUENCE_LAYDOWN"},
    {rumblex_interfaces::msg::MovementRequest::SEQUENCE_STAND_UP, "SEQUENCE_STAND_UP"},
    {rumblex_interfaces::msg::MovementRequest::SEQUENCE_WAITING, "SEQUENCE_WAITING"},
    {rumblex_interfaces::msg::MovementRequest::SEQUENCE_WATCH, "SEQUENCE_WATCH"},
    {rumblex_interfaces::msg::MovementRequest::SEQUENCE_LOOK, "SEQUENCE_LOOK"},
    {rumblex_interfaces::msg::MovementRequest::SEQUENCE_DANCE, "SEQUENCE_DANCE"},
    {rumblex_interfaces::msg::MovementRequest::SEQUENCE_HIGH_FIVE, "SEQUENCE_HIGH_FIVE"},
    {rumblex_interfaces::msg::MovementRequest::SEQUENCE_LEGS_WAVE, "SEQUENCE_LEGS_WAVE"},
    {rumblex_interfaces::msg::MovementRequest::SEQUENCE_BODY_ROLL, "SEQUENCE_BODY_ROLL"},
    {rumblex_interfaces::msg::MovementRequest::SEQUENCE_BITE, "SEQUENCE_BITE"},
    {rumblex_interfaces::msg::MovementRequest::SEQUENCE_STOMP, "SEQUENCE_STOMP"},
    {rumblex_interfaces::msg::MovementRequest::SEQUENCE_CLAP, "SEQUENCE_CLAP"},
    {rumblex_interfaces::msg::MovementRequest::CONTINUOUS_POSE, "CONTINUOUS_POSE"},
    {rumblex_interfaces::msg::MovementRequest::SINGLE_POSE, "SINGLE_POSE"},
    {rumblex_interfaces::msg::MovementRequest::SEQUENCE_TESTLEGS, "SEQUENCE_TESTLEGS"},
    {rumblex_interfaces::msg::MovementRequest::CONTINUOUS_MOVE, "CONTINUOUS_MOVE"},
    {rumblex_interfaces::msg::MovementRequest::CONTINUOUS_RUNNING, "CONTINUOUS_RUNNING"},
};

// Auto-generated reverse map from movementTypeToName
inline const auto nameToMovementType = [] {
    std::map<const std::string, uint32_t> result;
    for (const auto& [key, val] : movementTypeToName) {
        result[val] = key;
    }
    return result;
}();

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 2) {
    return std::format("{:.{}f}", a_value, n);
}
}  // namespace brain