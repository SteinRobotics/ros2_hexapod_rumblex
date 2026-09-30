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
    angles.torso_coxa = 0.0 * units::deg;
    angles.coxa_femur = 2.276 * units::deg;
    angles.femur_tibia = 7.704 * units::deg;

    kin_->setLegAngles(ELegIndex::RightFront, angles);

    CPosition toePosExpected;
    toePosExpected.x = 0.201 * units::m;   // 0.092 + 0.109
    toePosExpected.y = 0.160 * units::m;   // 0.092 + 0.068
    toePosExpected.z = -0.050 * units::m;  // 0.045 - 0.095

    auto& leg = kin_->getLegs().at(ELegIndex::RightFront);

    expectPositionNear(toePosExpected, leg.toe_pos_, "setLegAngles position mismatch");
    expectAnglesNear(angles, leg.angles_, "setLegAngles angles mismatch");
}

TEST_F(KinematicsTest, checkStandingPosition) {
    kin_->moveTorso(kin_->getLegsStandingPositions());
    auto& leg = kin_->getLegs().at(ELegIndex::RightFront);

    CPosition toePosExpected;
    toePosExpected.x = 0.201 * units::m;   // 0.092 + 0.109
    toePosExpected.y = 0.160 * units::m;   // 0.092 + 0.068
    toePosExpected.z = -0.050 * units::m;  // 0.045 - 0.095

    CLegAngles anglesExpected;
    anglesExpected.torso_coxa = 0.0 * units::deg;
    anglesExpected.coxa_femur = 2.276 * units::deg;
    anglesExpected.femur_tibia = 7.704 * units::deg;

    expectPositionNear(toePosExpected, leg.toe_pos_, "standing position mismatch");
    expectAnglesNear(anglesExpected, leg.angles_, "standing angles mismatch");
}

TEST_F(KinematicsTest, checkLaydownPosition) {
    kin_->moveTorso(kin_->getLegsLayDownPositions());
    auto& leg = kin_->getLegs().at(ELegIndex::RightFront);

    CPosition toePosExpected;
    toePosExpected.x = 0.180 * units::m;  // 0.071 + 0.109
    toePosExpected.y = 0.139 * units::m;  // 0.071 + 0.068
    toePosExpected.z = 0.010 * units::m;  // 0.045 - 0.035

    CLegAngles anglesExpected;
    anglesExpected.torso_coxa = 0.0 * units::deg;
    anglesExpected.coxa_femur = 70.723 * units::deg;
    anglesExpected.femur_tibia = -53.320 * units::deg;

    expectPositionNear(toePosExpected, leg.toe_pos_, "laydown position mismatch");
    expectAnglesNear(anglesExpected, leg.angles_, "laydown angles mismatch");
}

TEST_F(KinematicsTest, checkSetToe) {
    CPosition targetPos;
    targetPos.x = 0.201 * units::m;   // 0.092 + 0.109
    targetPos.y = 0.160 * units::m;   // 0.092 + 0.068
    targetPos.z = -0.050 * units::m;  // 0.045 - 0.095

    kin_->setSingleToe(ELegIndex::RightFront, targetPos);
    auto& leg = kin_->getLegs().at(ELegIndex::RightFront);

    CLegAngles anglesExpected;
    anglesExpected.torso_coxa = 0.0 * units::deg;
    anglesExpected.coxa_femur = 2.276 * units::deg;
    anglesExpected.femur_tibia = 7.704 * units::deg;

    expectPositionNear(targetPos, leg.toe_pos_, "setToe position mismatch");
    expectAnglesNear(anglesExpected, leg.angles_, "setToe angles mismatch");
}

TEST_F(KinematicsTest, TorsoRotationPreservesLegLocalTarget) {
    kin_->moveTorso(kin_->getLegsStandingPositions());
    const auto standingAngles = kin_->getAngles(ELegIndex::RightFront);

    // Rotate the coxa mount by 90 degrees about each axis and keep the same
    // leg-local toe target: (92, 92, -50) mm relative to the mount.
    const std::array<std::pair<COrientation, CPosition>, 3> cases = {{
        {COrientation(90.0 * units::deg, 0.0 * units::rad, 0.0 * units::rad),
         CPosition(201.0 * units::mm, 92.0 * units::mm, 18.0 * units::mm)},
        {COrientation(0.0 * units::rad, 90.0 * units::deg, 0.0 * units::rad),
         CPosition(92.0 * units::mm, 160.0 * units::mm, -159.0 * units::mm)},
        {COrientation(0.0 * units::rad, 0.0 * units::rad, 90.0 * units::deg),
         CPosition(24.0 * units::mm, 201.0 * units::mm, -50.0 * units::mm)},
    }};

    for (const auto& [orientation, target] : cases) {
        kin_->moveTorso({{ELegIndex::RightFront, target}}, CPose(CPosition(), orientation));
        expectAnglesNear(standingAngles, kin_->getAngles(ELegIndex::RightFront),
                         "rotated mount changed the leg-local target");
    }
}
