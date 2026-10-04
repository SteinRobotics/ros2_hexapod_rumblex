/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include <functional>

#include "rclcpp/rclcpp.hpp"
//
#include "rumblex_interfaces/msg/servo_index.hpp"
//

#include "rumblex_utils/body_types.hpp"
#include "servo_controller.hpp"

namespace rumblex_movement {

class CRequest {
   public:
    CRequest() = default;
    CRequest(rumblex_geometry::COrientation head,
             std::map<rumblex_geometry::ELegIndex, rumblex_geometry::CLegAngles> leg_angles, double duration)
        : head_orientation_(head), leg_angles_(std::move(leg_angles)), duration_(duration) {};

    ~CRequest() = default;

    const rumblex_geometry::COrientation& getHeadOrientation() const {
        return head_orientation_;
    }
    const std::map<rumblex_geometry::ELegIndex, rumblex_geometry::CLegAngles>& getLegAngles() const {
        return leg_angles_;
    }
    double duration() const {
        return duration_;
    }

   private:
    rumblex_geometry::COrientation head_orientation_;
    std::map<rumblex_geometry::ELegIndex, rumblex_geometry::CLegAngles> leg_angles_;
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
