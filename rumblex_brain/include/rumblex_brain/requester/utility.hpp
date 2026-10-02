/*******************************************************************************
 * Copyright (c) 2025 Christian Stein
 ******************************************************************************/

#pragma once

#include <algorithm>
#include <format>
#include <map>
#include <string>

#include "movement_request.hpp"

namespace brain {

const std::map<const uint32_t, const std::string> movementTypeToName = {
    {brain::MovementRequest::NO_REQUEST, "NO_REQUEST"},
    {brain::MovementRequest::SEQUENCE_LAYDOWN, "SEQUENCE_LAYDOWN"},
    {brain::MovementRequest::SEQUENCE_STAND_UP, "SEQUENCE_STAND_UP"},
    {brain::MovementRequest::SEQUENCE_WAITING, "SEQUENCE_WAITING"},
    {brain::MovementRequest::SEQUENCE_WATCH, "SEQUENCE_WATCH"},
    {brain::MovementRequest::SEQUENCE_LOOK, "SEQUENCE_LOOK"},
    {brain::MovementRequest::SEQUENCE_DANCE, "SEQUENCE_DANCE"},
    {brain::MovementRequest::SEQUENCE_HIGH_FIVE, "SEQUENCE_HIGH_FIVE"},
    {brain::MovementRequest::SEQUENCE_LEGS_WAVE, "SEQUENCE_LEGS_WAVE"},
    {brain::MovementRequest::SEQUENCE_BODY_ROLL, "SEQUENCE_BODY_ROLL"},
    {brain::MovementRequest::SEQUENCE_BITE, "SEQUENCE_BITE"},
    {brain::MovementRequest::SEQUENCE_STOMP, "SEQUENCE_STOMP"},
    {brain::MovementRequest::SEQUENCE_CLAP, "SEQUENCE_CLAP"},
    {brain::MovementRequest::CONTINUOUS_POSE, "CONTINUOUS_POSE"},
    {brain::MovementRequest::SINGLE_POSE, "SINGLE_POSE"},
    {brain::MovementRequest::SEQUENCE_TESTLEGS, "SEQUENCE_TESTLEGS"},
    {brain::MovementRequest::CONTINUOUS_MOVE, "CONTINUOUS_MOVE"},
    {brain::MovementRequest::CONTINUOUS_RUNNING, "CONTINUOUS_RUNNING"},
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