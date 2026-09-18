#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "requester/kinematics.hpp"
#include "test_helpers.hpp"

using namespace std;
using namespace rumblex_movement;

class KinematicsTest : public ::testing::Test {
   protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        rclcpp::NodeOptions options;
        auto overrides = test_helpers::defaultRobotParameters();
        options.parameter_overrides(overrides);
        node_ = std::make_shared<rclcpp::Node>("test_kinematics_node", options);
        kin_ = std::make_unique<CKinematics>(node_);
        cout << "KinematicsTest SetUp complete" << endl;
    }

    void TearDown() override {
        kin_.reset();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<CKinematics> kin_;
};

// Standing position
// RightFront: 	ag: 0.000°, 2.276°, 7.704°	| x: 0.201, y: 0.160, z: -0.050

// Laydown position
// RightFront: 	ag: 0.000°, 70.723°, -53.320°	| x: 0.180, y: 0.139, z: 0.010

TEST_F(KinematicsTest, setLegAngles) {
    // create target angles
    CLegAngles angles;
    angles.coxa_deg = 0.0;
    angles.femur_deg = 2.276;
    angles.tibia_deg = 7.704;

    kin_->setLegAngles(ELegIndex::RightFront, angles);

    CPosition footPosExpected;
    footPosExpected.x = 0.201;   // 0.092 + 0.109
    footPosExpected.y = 0.160;   // 0.092 + 0.068
    footPosExpected.z = -0.050;  // 0.045 - 0.095

    auto& leg = kin_->getLegs().at(ELegIndex::RightFront);

    expectPositionNear(footPosExpected, leg.foot_pos_, "setLegAngles position mismatch");
    expectAnglesNear(angles, leg.angles_deg_, "setLegAngles angles mismatch");
}

TEST_F(KinematicsTest, checkStandingPosition) {
    kin_->moveBody(kin_->getLegsStandingPositions());
    auto& leg = kin_->getLegs().at(ELegIndex::RightFront);

    CPosition footPosExpected;
    footPosExpected.x = 0.201;   // 0.092 + 0.109
    footPosExpected.y = 0.160;   // 0.092 + 0.068
    footPosExpected.z = -0.050;  // 0.045 - 0.095

    CLegAngles anglesExpected;
    anglesExpected.coxa_deg = 0.0;
    anglesExpected.femur_deg = 2.276;
    anglesExpected.tibia_deg = 7.704;

    expectPositionNear(footPosExpected, leg.foot_pos_, "standing position mismatch");
    expectAnglesNear(anglesExpected, leg.angles_deg_, "standing angles mismatch");
}

TEST_F(KinematicsTest, checkLaydownPosition) {
    kin_->moveBody(kin_->getLegsLayDownPositions());
    auto& leg = kin_->getLegs().at(ELegIndex::RightFront);

    CPosition footPosExpected;
    footPosExpected.x = 0.180;  // 0.071 + 0.109
    footPosExpected.y = 0.139;  // 0.071 + 0.068
    footPosExpected.z = 0.010;  // 0.045 - 0.035

    CLegAngles anglesExpected;
    anglesExpected.coxa_deg = 0.0;
    anglesExpected.femur_deg = 70.723;
    anglesExpected.tibia_deg = -53.320;

    expectPositionNear(footPosExpected, leg.foot_pos_, "laydown position mismatch");
    expectAnglesNear(anglesExpected, leg.angles_deg_, "laydown angles mismatch");
}

TEST_F(KinematicsTest, checkSetFeet) {
    CPosition targetPos;
    targetPos.x = 0.201;   // 0.092 + 0.109
    targetPos.y = 0.160;   // 0.092 + 0.068
    targetPos.z = -0.050;  // 0.045 - 0.095

    kin_->setSingleFeet(ELegIndex::RightFront, targetPos);
    auto& leg = kin_->getLegs().at(ELegIndex::RightFront);

    CLegAngles anglesExpected;
    anglesExpected.coxa_deg = 0.0;
    anglesExpected.femur_deg = 2.276;
    anglesExpected.tibia_deg = 7.704;

    expectPositionNear(targetPos, leg.foot_pos_, "setFeet position mismatch");
    expectAnglesNear(anglesExpected, leg.angles_deg_, "setFeet angles mismatch");
}