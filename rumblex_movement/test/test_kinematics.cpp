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

    CPosition toe_pos_expected;
    toe_pos_expected.x = 0.201 * units::m;   // 0.092 + 0.109
    toe_pos_expected.y = 0.160 * units::m;   // 0.092 + 0.068
    toe_pos_expected.z = -0.050 * units::m;  // 0.045 - 0.095

    auto& leg = kin_->getLegs().at(ELegIndex::RightFront);

    expectPositionNear(toe_pos_expected, leg.toe_position, "setLegAngles position mismatch");
    expectAnglesNear(angles, leg.angles, "setLegAngles angles mismatch");
}

TEST_F(KinematicsTest, check_standing_position) {
    kin_->moveTorso(kin_->getStandingToePositions());
    auto& leg = kin_->getLegs().at(ELegIndex::RightFront);

    CPosition toe_pos_expected;
    toe_pos_expected.x = 0.201 * units::m;   // 0.092 + 0.109
    toe_pos_expected.y = 0.160 * units::m;   // 0.092 + 0.068
    toe_pos_expected.z = -0.050 * units::m;  // 0.045 - 0.095

    CLegAngles angles_expected;
    angles_expected.torso_coxa = 0.0 * units::deg;
    angles_expected.coxa_femur = 2.276 * units::deg;
    angles_expected.femur_tibia = 7.704 * units::deg;

    expectPositionNear(toe_pos_expected, leg.toe_position, "standing position mismatch");
    expectAnglesNear(angles_expected, leg.angles, "standing angles mismatch");
}

TEST_F(KinematicsTest, check_laydown_position) {
    kin_->moveTorso(kin_->getLaydownToePositions());
    auto& leg = kin_->getLegs().at(ELegIndex::RightFront);

    CPosition toe_pos_expected;
    toe_pos_expected.x = 0.180 * units::m;  // 0.071 + 0.109
    toe_pos_expected.y = 0.139 * units::m;  // 0.071 + 0.068
    toe_pos_expected.z = 0.010 * units::m;  // 0.045 - 0.035

    CLegAngles angles_expected;
    angles_expected.torso_coxa = 0.0 * units::deg;
    angles_expected.coxa_femur = 70.723 * units::deg;
    angles_expected.femur_tibia = -53.320 * units::deg;

    expectPositionNear(toe_pos_expected, leg.toe_position, "laydown position mismatch");
    expectAnglesNear(angles_expected, leg.angles, "laydown angles mismatch");
}

TEST_F(KinematicsTest, check_set_toe) {
    CPosition target_pos;
    target_pos.x = 0.201 * units::m;   // 0.092 + 0.109
    target_pos.y = 0.160 * units::m;   // 0.092 + 0.068
    target_pos.z = -0.050 * units::m;  // 0.045 - 0.095

    kin_->setToePosition(ELegIndex::RightFront, target_pos);
    auto& leg = kin_->getLegs().at(ELegIndex::RightFront);

    CLegAngles angles_expected;
    angles_expected.torso_coxa = 0.0 * units::deg;
    angles_expected.coxa_femur = 2.276 * units::deg;
    angles_expected.femur_tibia = 7.704 * units::deg;

    expectPositionNear(target_pos, leg.toe_position, "setToe position mismatch");
    expectAnglesNear(angles_expected, leg.angles, "setToe angles mismatch");
}

TEST_F(KinematicsTest, TorsoRotationPreservesLegLocalTarget) {
    kin_->moveTorso(kin_->getStandingToePositions());
    const auto standing_angles = kin_->getLegAngles(ELegIndex::RightFront);

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
        expectAnglesNear(standing_angles, kin_->getLegAngles(ELegIndex::RightFront),
                         "rotated mount changed the leg-local target");
    }
}
