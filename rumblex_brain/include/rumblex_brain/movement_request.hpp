#pragma once
#include <cstdint>
#include <string>
namespace brain {
struct MovementRequest {
    enum Type : uint8_t {
        NO_REQUEST = 0,
        SINGLE_POSE = 1,
        SEQUENCE_LAYDOWN = 10,
        SEQUENCE_STAND_UP = 11,
        SEQUENCE_WAITING = 12,
        SEQUENCE_WATCH = 13,
        SEQUENCE_LOOK = 14,
        SEQUENCE_DANCE = 15,
        SEQUENCE_HIGH_FIVE = 16,
        SEQUENCE_LEGS_WAVE = 17,
        SEQUENCE_BODY_ROLL = 18,
        SEQUENCE_BITE = 19,
        SEQUENCE_STOMP = 20,
        SEQUENCE_CLAP = 21,
        SEQUENCE_TESTLEGS = 22,
        CONTINUOUS_POSE = 30,
        CONTINUOUS_MOVE = 31,
        CONTINUOUS_RUNNING = 32,
    };
    enum Direction : uint8_t { CLOCKWISE = 0, ANTICLOCKWISE = 1 };
    using _type_type = uint8_t;
    uint8_t type = NO_REQUEST;
    uint8_t direction = CLOCKWISE;
    double duration_s = 1.0;
    std::string name;
};
}  // namespace brain
