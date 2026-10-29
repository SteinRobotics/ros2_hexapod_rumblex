/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "movement_request.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rumblex_interfaces/msg/pose.hpp"

namespace brain {

using namespace mp_units;
using namespace mp_units::si::unit_symbols;

enum class Prio {
    // is executed immediately, stops all other requests, cancel all other/pending/queued requests
    Highest = 0,
    // is executed immediately, stops all other requests, other request are continued after this request has finished
    High = 1,
    // is queued and executed as soon as no other request is active
    Normal = 2,
    // is executed only if no other pending request exists, only one (the latest) Request with Prio Background can exist. It is stored and can only be set to OFF by highest Prio.
    Background = 3,
};

struct RequestBase {
    units::Duration minDuration = 0.0 * s;
    virtual ~RequestBase() = default;
};

struct RequestSystem : RequestBase {
    bool turnOffServoRelay = false;
    bool systemShutdown = false;
};

struct RequestTalking : RequestBase {
    std::string text;
    std::string language = "de";
};

struct RequestChat : RequestBase {
    std::string text;
    std::string language = "de";
};

struct RequestMusic : RequestBase {
    std::string song;
    float volume = 0.8f;
};

struct RequestListening : RequestBase {
    bool active = false;
};

struct RequestMovementType : RequestBase {
    brain::MovementRequest movementRequest;
};

struct RequestVelocity : RequestBase {
    geometry_msgs::msg::Twist velocity;
};

}  // namespace brain
