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
    CRequest(COrientation head, std::map<ELegIndex, CLegAngles> legAngles, double duration)
        : head_(head), legAngles_(std::move(legAngles)), duration_(duration) {};

    ~CRequest() = default;

    const COrientation& head() const {
        return head_;
    }
    const std::map<ELegIndex, CLegAngles>& legAngles() const {
        return legAngles_;
    }
    double duration() const {
        return duration_;
    }

   private:
    COrientation head_;
    std::map<ELegIndex, CLegAngles> legAngles_;
    double duration_ = double(0);
};

class CServoHandler {
   public:
    CServoHandler(std::shared_ptr<rclcpp::Node> node);
    virtual ~CServoHandler() = default;

    virtual void run(CRequest request);

    std::shared_ptr<CServoController> getServoController() const {
        return servoController_;
    }

   private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CServoController> servoController_;
};

}  // namespace rumblex_movement
