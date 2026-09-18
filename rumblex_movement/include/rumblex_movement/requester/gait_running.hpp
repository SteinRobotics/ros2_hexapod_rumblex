/*******************************************************************************
 * Copyright (c) 2026 Christian Stein
 *
 * Running gait – fastest locomotion pattern.
 * Only 2 legs (one diagonal pair from a tripod group) touch the ground at a
 * time while the other 4 are in the air.  A brief "flight phase" where both
 * tripod groups are airborne makes this a true run rather than a walk.
 * Because the robot is statically unstable, the flight fraction is kept short.
 ******************************************************************************/

#pragma once

#include <array>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "rumblex_utils/filters.hpp"
#include "rumblex_utils/geometry.hpp"
#include "rumblex_utils/msg_twist.hpp"
#include "rumblex_utils/simpletimer.hpp"
#include "requester/gait_parameters.hpp"
#include "requester/igaits.hpp"
#include "requester/kinematics.hpp"
#include "requester/types.hpp"

namespace rumblex_movement {

class CGaitRunning : public IContinuousGait {
   public:
    CGaitRunning(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                 Parameters::Running& params);
    ~CGaitRunning() override = default;

    void start(double duration_s, uint8_t direction) override;
    bool update(const geometry_msgs::msg::Twist& velocity, const CPose& body,
                const COrientation& head) override;
    void requestStop() override;
    void cancelStop() override;
    EGaitState state() const override {
        return state_;
    }

   private:
    struct LegMotion {
        double step{0.0};
        double lift{0.0};
    };

    LegMotion computeLegMotion(ELegIndex index, double phase) const;

    // Two tripod groups – identical to the tripod gait grouping
    static constexpr std::array<ELegIndex, 3> kGroupA = {ELegIndex::RightFront, ELegIndex::LeftMid,
                                                         ELegIndex::RightBack};

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;
    Parameters::Running params_;

    EGaitState state_ = EGaitState::Stopped;
    double phase_ = 0.0;

    CSimpleTimer no_velocity_timer_;
    std::map<ELegIndex, CPosition> target_positions_;
    CPose body_old_;

    geometry_msgs::msg::Twist velocity_{geometry_msgs::msg::Twist()};
};

}  // namespace rumblex_movement
