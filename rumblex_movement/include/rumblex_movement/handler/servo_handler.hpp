/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include <functional>

#include "rclcpp/rclcpp.hpp"
//
#include "rumblex_interfaces/msg/servo_index.hpp"
//

#include "requester/kinematics.hpp"
#include "servo_controller.hpp"

namespace rumblex_movement {

class CRequest {
   public:
    CRequest() = default;
    CRequest(COrientation head, std::map<ELegIndex, CLegAngles> leg_angles, double duration)
        : head_orientation_(head), leg_angles_(std::move(leg_angles)), duration_(duration) {};

    ~CRequest() = default;

    const COrientation& getHeadOrientation() const {
        return head_orientation_;
    }
    const std::map<ELegIndex, CLegAngles>& getLegAngles() const {
        return leg_angles_;
    }
    double duration() const {
        return duration_;
    }

   private:
    COrientation head_orientation_;
    std::map<ELegIndex, CLegAngles> leg_angles_;
    double duration_ = double(0);
};

class CServoHandler {
   public:
    CServoHandler(std::shared_ptr<rclcpp::Node> node);
    virtual ~CServoHandler() = default;

    virtual void run(CRequest request);

    std::shared_ptr<CServoController> getServoController() const {
        return servo_controller_;
    }

   private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CServoController> servo_controller_;
};

}  // namespace rumblex_movement
