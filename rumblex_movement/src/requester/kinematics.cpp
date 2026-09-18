/*******************************************************************************
 * Copyright (c) 2024 Christian Stein
 ******************************************************************************/

#include "requester/kinematics.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

#include "sensor_msgs/msg/joint_state.hpp"

using namespace std;
using namespace utils;

namespace rumblex_movement {

#define LOG_KINEMATICS_LEG_ACTIVE false
#define LOG_KINEMATICS_HEAD_ACTIVE false

CKinematics::CKinematics(std::shared_ptr<rclcpp::Node> node)
    : node_(node),
      COXA_LENGTH(node->declare_parameter<double>("COXA_LENGTH", rclcpp::PARAMETER_DOUBLE)),
      COXA_HEIGHT(node->declare_parameter<double>("COXA_HEIGHT", rclcpp::PARAMETER_DOUBLE)),
      FEMUR_LENGTH(node->declare_parameter<double>("FEMUR_LENGTH", rclcpp::PARAMETER_DOUBLE)),
      TIBIA_LENGTH(node->declare_parameter<double>("TIBIA_LENGTH", rclcpp::PARAMETER_DOUBLE)),
      sq_femur_length_(pow(FEMUR_LENGTH, 2)),
      sq_tibia_length_(pow(TIBIA_LENGTH, 2)) {
    std::map<ELegIndex, std::string> leg_parameter_keys;
    for (auto leg_index : magic_enum::enum_values<ELegIndex>()) {
        const std::string canonical_name = std::string(magic_enum::enum_name(leg_index));
        std::string parameter_suffix =
            node_->declare_parameter<std::string>("leg_names." + canonical_name, canonical_name);
        leg_parameter_keys[leg_index] = parameter_suffix;
    }

    auto loadBodyCenterOffsetsFromParameters =
        [&](const std::string& parameter_root, const std::map<ELegIndex, std::string>& leg_parameter_names) {
            std::map<ELegIndex, CBodyCenterOffset> offsets;

            for (const auto& [leg_index, parameter_suffix] : leg_parameter_names) {
                const std::string prefix = parameter_root + "." + parameter_suffix;
                CBodyCenterOffset offset;
                offset.x = node_->declare_parameter<double>(prefix + ".CENTER_TO_COXA_X", 0.0);
                offset.y = node_->declare_parameter<double>(prefix + ".CENTER_TO_COXA_Y", 0.0);
                offset.psi_deg = node_->declare_parameter<double>(prefix + ".OFFSET_COXA_ANGLE_DEG", 0.0);
                offsets[leg_index] = offset;
            }

            RCLCPP_INFO_STREAM(node_->get_logger(), "Loaded " << parameter_root
                                                              << " offsets from parameters ("
                                                              << offsets.size() << " entries).");
            return offsets;
        };

    auto loadFootPositionsFromParameters = [&](const std::string& parameter_root,
                                               const std::map<ELegIndex, std::string>& leg_parameter_names) {
        std::map<ELegIndex, CPosition> positions;
        for (const auto& [leg_index, parameter_suffix] : leg_parameter_names) {
            const std::string prefix = parameter_root + "." + parameter_suffix;
            positions[leg_index] = CPosition(node_->declare_parameter<double>(prefix + ".x", 0.0),
                                             node_->declare_parameter<double>(prefix + ".y", 0.0),
                                             node_->declare_parameter<double>(prefix + ".z", 0.0));
        }
        return positions;
    };

    bodyCenterOffsets_ = loadBodyCenterOffsetsFromParameters("leg_offsets", leg_parameter_keys);

    const auto foot_positions_standing =
        loadFootPositionsFromParameters("footPositions_standing", leg_parameter_keys);
    initializeLegs(foot_positions_standing, body_, legsStanding_);

    const auto foot_positions_laydown =
        loadFootPositionsFromParameters("footPositions_laydown", leg_parameter_keys);
    initializeLegs(foot_positions_laydown, body_, legsLayDown_);

    // Initialize current leg positions to laydown (robot starts laying down)
    initializeLegs(foot_positions_laydown, body_, legs_);
}

void CKinematics::logLegsPositions(std::map<ELegIndex, CLeg>& legs) {
    if (!LOG_KINEMATICS_LEG_ACTIVE) return;
    RCLCPP_INFO_STREAM(node_->get_logger(), "----------------------------------------");
    for (const auto& [index, leg] : legs) {
        logLegPosition(index, leg);
    }
    RCLCPP_INFO_STREAM(node_->get_logger(), "----------------------------------------");
}

void CKinematics::logLegPosition(const ELegIndex index, const CLeg& leg) {
    if (!LOG_KINEMATICS_LEG_ACTIVE) return;
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       magic_enum::enum_name(index)
                           << ": \tag: " << std::fixed << std::setprecision(3) << std::setw(3)
                           << leg.angles_deg_.coxa_deg << "°, " << std::setw(3) << leg.angles_deg_.femur_deg
                           << "°, " << std::setw(3) << leg.angles_deg_.tibia_deg << "°\t| x: " << std::fixed
                           << std::setprecision(3) << std::setw(3) << leg.foot_pos_.x << ", y: "
                           << std::setw(3) << leg.foot_pos_.y << ", z: " << std::setw(3) << leg.foot_pos_.z);
}

void CKinematics::logHeadPosition() {
    if (!LOG_KINEMATICS_HEAD_ACTIVE) return;
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "Head: \tYaw: " << std::fixed << std::setprecision(3) << std::setw(3) << head_.yaw_deg
                                       << "°, Pitch: " << std::setw(3) << head_.pitch_deg << "°");
}

void CKinematics::initializeLegs(const std::map<ELegIndex, CPosition>& footTargets, const CPose body,
                                 std::map<ELegIndex, CLeg>& legs) {
    for (const auto& [leg_index, foot_target] : footTargets) {
        RCLCPP_DEBUG_STREAM(node_->get_logger(), "Initializing leg "
                                                     << magic_enum::enum_name(leg_index)
                                                     << " to foot target position x: " << foot_target.x
                                                     << ", y: " << foot_target.y << ", z: " << foot_target.z);
        auto leg = CLeg();
        CPosition coxa_position(bodyCenterOffsets_.at(leg_index).x, bodyCenterOffsets_.at(leg_index).y, 0.0);
        CPosition leg_base = rotate(coxa_position, body.orientation) + body.position;
        CPosition foot_rel = foot_target - leg_base;
        calcLegInverseKinematics(foot_rel, leg, leg_index);
        legs[leg_index] = leg;
    }
    logLegsPositions(legs);
}

void CKinematics::moveBody(const std::map<ELegIndex, CPosition>& foot_targets, const CPose body) {
    body_ = body;

    for (auto& [leg_index, foot_target] : foot_targets) {
        auto& leg = legs_.at(leg_index);
        CPosition coxa_position(bodyCenterOffsets_.at(leg_index).x, bodyCenterOffsets_.at(leg_index).y, 0.0);
        CPosition leg_base = rotate(coxa_position, body.orientation) + body.position;
        CPosition foot_rel = foot_target - leg_base;
        calcLegInverseKinematics(foot_rel, leg, leg_index);
    }
    logLegsPositions(legs_);
}

void CKinematics::moveBody(const CPose body) {
    body_ = body;

    auto foot_targets = getLegsStandingPositions();

    for (auto& [leg_index, foot_target] : foot_targets) {
        auto& leg = legs_.at(leg_index);
        CPosition coxa_position(bodyCenterOffsets_.at(leg_index).x, bodyCenterOffsets_.at(leg_index).y, 0.0);
        CPosition leg_base = rotate(coxa_position, body.orientation) + body.position;
        CPosition foot_rel = foot_target - leg_base;
        calcLegInverseKinematics(foot_rel, leg, leg_index);
    }
    logLegsPositions(legs_);
}

CPosition CKinematics::rotate(const CPosition& point, const COrientation& orientation) {
    double px = point.x;
    double py = point.y;
    double pz = point.z;

    // Rotation angles are in degrees, convert to radians
    double rollRad = deg2rad(orientation.roll_deg);
    double pitchRad = deg2rad(orientation.pitch_deg);
    double yawRad = deg2rad(orientation.yaw_deg);

    double cosRoll = cos(rollRad);
    double sinRoll = sin(rollRad);
    double cosPitch = cos(pitchRad);
    double sinPitch = sin(pitchRad);
    double cosYaw = cos(yawRad);
    double sinYaw = sin(yawRad);

    // Standard ZYX (yaw-pitch-roll) rotation matrix
    double rotatedX = cosYaw * cosPitch * px + (cosYaw * sinPitch * sinRoll - sinYaw * cosRoll) * py +
                      (cosYaw * sinPitch * cosRoll + sinYaw * sinRoll) * pz;

    double rotatedY = sinYaw * cosPitch * px + (sinYaw * sinPitch * sinRoll + cosYaw * cosRoll) * py +
                      (sinYaw * sinPitch * cosRoll - cosYaw * sinRoll) * pz;

    double rotatedZ = -sinPitch * px + cosPitch * sinRoll * py + cosPitch * cosRoll * pz;

    return {rotatedX, rotatedY, rotatedZ};
}

void CKinematics::calcLegInverseKinematics(const CPosition& targetFeetPos, CLeg& leg,
                                           const ELegIndex& legIndex) {
    leg.foot_pos_ = targetFeetPos;

    double agCoxaRad = atan2(targetFeetPos.x, targetFeetPos.y);
    double zOffset = COXA_HEIGHT - targetFeetPos.z;

    double lLegTopView = std::hypot(targetFeetPos.x, targetFeetPos.y);  // L1

    double sqL = pow(zOffset, 2) + pow(lLegTopView - COXA_LENGTH, 2);
    double L = sqrt(sqL);

    double tmpFemur = (sq_tibia_length_ - sq_femur_length_ - sqL) / (-2 * FEMUR_LENGTH * L);
    if (abs(tmpFemur) > 1.0) {
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                            "calcLegInverseKinematics: clamping femur input "
                                << tmpFemur << " for leg " << magic_enum::enum_name(legIndex)
                                << " (target x: " << targetFeetPos.x << ", y: " << targetFeetPos.y
                                << ", z: " << targetFeetPos.z << ")");
        tmpFemur = std::clamp(tmpFemur, -1.0, 1.0);
    }
    double agFemurRad = acos(zOffset / L) + acos(tmpFemur) - (M_PI / 2);

    double tmpTibia = (sqL - sq_tibia_length_ - sq_femur_length_) / (-2 * FEMUR_LENGTH * TIBIA_LENGTH);
    if (abs(tmpTibia) > 1.0) {
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                            "calcLegInverseKinematics: clamping tibia input "
                                << tmpTibia << " for leg " << magic_enum::enum_name(legIndex)
                                << " (target x: " << targetFeetPos.x << ", y: " << targetFeetPos.y
                                << ", z: " << targetFeetPos.z << ")");
        tmpTibia = std::clamp(tmpTibia, -1.0, 1.0);
    }
    double agTibiaRad = acos(tmpTibia) - (M_PI / 2);

    leg.angles_deg_.coxa_deg = float(rad2deg(agCoxaRad - (M_PI / 2)));  // TODO why -90?
    leg.angles_deg_.femur_deg = float(rad2deg(agFemurRad));
    leg.angles_deg_.tibia_deg = float(rad2deg(agTibiaRad));

    // for leg local coordinate system we have to add the body center offset
    leg.angles_deg_.coxa_deg += bodyCenterOffsets_[legIndex].psi_deg;

    if (leg.angles_deg_.coxa_deg > 180.0) {
        leg.angles_deg_.coxa_deg -= 360.0;
    } else if (leg.angles_deg_.coxa_deg < -180.0) {
        leg.angles_deg_.coxa_deg += 360.0;
    }

    leg.foot_pos_.x += bodyCenterOffsets_[legIndex].x;
    leg.foot_pos_.y += bodyCenterOffsets_[legIndex].y;
}

void CKinematics::calcLegForwardKinematics(const CLegAngles target, CLeg& leg) {
    leg.angles_deg_ = target;

    leg.foot_pos_.x = (COXA_LENGTH + FEMUR_LENGTH * cos(deg2rad(target.femur_deg)) +
                       TIBIA_LENGTH * cos(deg2rad(-90 + target.femur_deg + target.tibia_deg))) *
                      sin(deg2rad(90 + target.coxa_deg));

    leg.foot_pos_.y = (COXA_LENGTH + FEMUR_LENGTH * cos(deg2rad(target.femur_deg)) +
                       TIBIA_LENGTH * cos(deg2rad(-90 + target.femur_deg + target.tibia_deg))) *
                      cos(deg2rad(90 + target.coxa_deg));

    leg.foot_pos_.z = COXA_HEIGHT + (FEMUR_LENGTH * sin(deg2rad(target.femur_deg)) +
                                     TIBIA_LENGTH * sin(deg2rad(-90 + target.femur_deg + target.tibia_deg)));
}

void CKinematics::setSingleFeet(const ELegIndex legIndex, const CPosition& targetFeetPos) {
    moveBody({{legIndex, targetFeetPos}}, body_);
}

void CKinematics::setLegAngles(const ELegIndex index, const CLegAngles& angles) {
    auto& leg = legs_.at(index);

    // transform to leg coordinate system by subtracting the angle psi
    CLegAngles targetAngles = angles;
    targetAngles.coxa_deg -= bodyCenterOffsets_[index].psi_deg;

    calcLegForwardKinematics(targetAngles, leg);

    // transform back to robot coordinate system by adding body center offset and adding psi
    leg.foot_pos_.x += bodyCenterOffsets_[index].x;
    leg.foot_pos_.y += bodyCenterOffsets_[index].y;
    leg.angles_deg_.coxa_deg += bodyCenterOffsets_[index].psi_deg;

    logLegPosition(index, leg);
}

void CKinematics::setHead(COrientation head) {
    head_ = head;
    logHeadPosition();
}

void CKinematics::setHead(double yaw_deg, double pitch_deg) {
    head_.yaw_deg = yaw_deg;
    head_.pitch_deg = pitch_deg;
    logHeadPosition();
}

std::map<ELegIndex, CLeg>& CKinematics::getLegs() {
    return legs_;
}

CLegAngles& CKinematics::getAngles(ELegIndex index) {
    return legs_.at(index).angles_deg_;
}

std::map<ELegIndex, CLegAngles> CKinematics::getLegsAngles() {
    std::map<ELegIndex, CLegAngles> legAngles;
    for (auto& [index, leg] : legs_) {
        legAngles[index] = leg.angles_deg_;
    }
    return legAngles;
}

std::map<ELegIndex, CPosition> CKinematics::getLegsPositions() const {
    std::map<ELegIndex, CPosition> positions;
    for (auto& [legIndex, leg] : legs_) {
        positions[legIndex] = leg.foot_pos_;
    }
    return positions;
}

std::map<ELegIndex, CPosition> CKinematics::getLegsStandingPositions() const {
    std::map<ELegIndex, CPosition> footTargets;
    for (auto& [legIndex, leg] : legsStanding_) {
        footTargets[legIndex] = leg.foot_pos_;
    }
    return footTargets;
}

std::map<ELegIndex, CPosition> CKinematics::getLegsLayDownPositions() const {
    std::map<ELegIndex, CPosition> footTargets;
    for (auto& [legIndex, leg] : legsLayDown_) {
        footTargets[legIndex] = leg.foot_pos_;
    }
    return footTargets;
}

}  // namespace rumblex_movement
