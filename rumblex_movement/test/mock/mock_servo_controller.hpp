#pragma once

#include <vector>

#include "handler/i_servo_controller.hpp"

namespace rumblex_movement {

class CServoControllerMock : public IServoController {
   public:
    struct Request {
        std::map<uint32_t, double> target_angles;
        double duration_s;
    };

    void requestAngles(const std::map<uint32_t, double>& target_angles, double duration_s) override {
        requests.push_back({target_angles, duration_s});
    }

    void setInitialAnglesCallback(InitialAnglesCallback callback) override {
        callback(initial_leg_angles, initial_head_orientation);
    }

    std::vector<Request> requests;
    std::map<rumblex_geometry::ELegIndex, rumblex_geometry::CLegAngles> initial_leg_angles;
    rumblex_geometry::COrientation initial_head_orientation;
};

}  // namespace rumblex_movement
