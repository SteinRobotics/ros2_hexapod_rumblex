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

    void setSingleToe(const ELegIndex index, const CPosition& targetToePos);
    void setLegAngles(const ELegIndex index, const CLegAngles& angles);
    void moveTorso(const std::map<ELegIndex, CPosition>& toeTargets,
                   const CPose torso = CPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    void moveTorso(const CPose torso);

    void setHead(units::Angle yaw, units::Angle pitch);
    void setHead(COrientation head);

    std::map<ELegIndex, CPosition> getLegsPositions() const;
    std::map<ELegIndex, CPosition> getLegsStandingPositions() const;
    std::map<ELegIndex, CPosition> getLegsLayDownPositions() const;

    std::map<ELegIndex, CLeg>& getLegs();
    CLegAngles& getAngles(ELegIndex index);
    std::map<ELegIndex, CLegAngles> getLegsAngles();

    COrientation& getHead() {
        return head_;
    };

    CPose& getTorso() {
        return torso_;
    };

   private:
    void initializeLegs(const std::map<ELegIndex, CPosition>& toeTargets, const CPose torso,
                        std::map<ELegIndex, CLeg>& legs);

    void logLegsPositions(std::map<ELegIndex, CLeg>& legs);
    void logLegPosition(const ELegIndex index, const CLeg& leg);
    void logHeadPosition();
    void calcLegInverseKinematics(const CPosition& targetToePos, CLeg& leg, const ELegIndex& legIndex);
    void calcLegForwardKinematics(const CLegAngles target, CLeg& leg);
    CPosition rotate(const CPosition& point, const COrientation& rot);

    std::shared_ptr<rclcpp::Node> node_;

    // Parameters
    const units::Length COXA_LENGTH;
    const units::Length COXA_HEIGHT;
    const units::Length FEMUR_LENGTH;
    const units::Length TIBIA_LENGTH;
    const units::Area sq_femur_length_;
    const units::Area sq_tibia_length_;

    std::map<ELegIndex, CLeg> legs_;          // current values
    std::map<ELegIndex, CLeg> legsStanding_;  // change to shared pointer and make const
    std::map<ELegIndex, CLeg> legsLayDown_;   // change to shared pointer and make const
    std::map<ELegIndex, CTorsoCenterOffset> torsoCenterOffsets_;

    CPose torso_ = {};
    COrientation head_ = {};
};

}  // namespace rumblex_movement
