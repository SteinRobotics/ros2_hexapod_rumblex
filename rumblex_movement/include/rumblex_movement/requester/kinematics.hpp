/*******************************************************************************
 * Copyright (c) 2024 Christian Stein
 ******************************************************************************/

#pragma once

#include <cassert>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "requester/types.hpp"
#include "rumblex_interfaces/msg/pose.hpp"
#include "rumblex_utils/geometry.hpp"
#include "units.hpp"

namespace rumblex_movement {

class CKinematics {
   public:
    explicit CKinematics(std::shared_ptr<rclcpp::Node> node);
    ~CKinematics() = default;

    void setToePosition(const ELegIndex index, const CPosition& target_toe_pos);
    void setLegAngles(const ELegIndex index, const CLegAngles& angles);
    void moveTorso(const std::map<ELegIndex, CPosition>& toe_targets,
                   const CPose torso = CPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    void moveTorso(const CPose torso);

    void setHeadOrientation(units::Angle yaw, units::Angle pitch);
    void setHeadOrientation(COrientation head);

    std::map<ELegIndex, CPosition> getToePositions() const;
    std::map<ELegIndex, CPosition> getStandingToePositions() const;
    std::map<ELegIndex, CPosition> getLaydownToePositions() const;

    std::map<ELegIndex, CLeg>& getLegs();
    CLegAngles& getLegAngles(ELegIndex index);
    std::map<ELegIndex, CLegAngles> getLegAngles();

    COrientation& getHeadOrientation() {
        return head_orientation_;
    };

    CPose& getTorsoPose() {
        return torso_pose_;
    };

   private:
    void initializeLegs(const std::map<ELegIndex, CPosition>& toe_targets, const CPose torso,
                        std::map<ELegIndex, CLeg>& legs);

    void logLegsPositions(std::map<ELegIndex, CLeg>& legs);
    void logLegPosition(const ELegIndex index, const CLeg& leg);
    void logHeadOrientation();
    void calcLegInverseKinematics(const CPosition& target_toe_pos, CLeg& leg, const ELegIndex& leg_index);
    void calcLegForwardKinematics(const CLegAngles target, CLeg& leg);
    CPosition rotate(const CPosition& point, const COrientation& rot);

    std::shared_ptr<rclcpp::Node> node_;

    // Parameters
    const units::Length coxa_length_;
    const units::Length coxa_height_;
    const units::Length femur_length_;
    const units::Length tibia_length_;
    const units::Area sq_femur_length_;
    const units::Area sq_tibia_length_;

    std::map<ELegIndex, CLeg> legs_;           // current values
    std::map<ELegIndex, CLeg> standing_legs_;  // change to shared pointer and make const
    std::map<ELegIndex, CLeg> laydown_legs_;   // change to shared pointer and make const
    std::map<ELegIndex, CTorsoCenterOffset> torso_center_offsets_;

    CPose torso_pose_ = {};
    COrientation head_orientation_ = {};
};

}  // namespace rumblex_movement
