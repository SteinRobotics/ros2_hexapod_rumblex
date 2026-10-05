#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "rumblex_utils/body_model.hpp"
#include "test_helpers.hpp"

using namespace std;

class KinematicsTest : public ::testing::Test {
   protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        rclcpp::NodeOptions options;
        auto overrides = rumblex_movement::test_helpers::defaultRobotParameters();
        options.parameter_overrides(overrides);
        node_ = std::make_shared<rclcpp::Node>("test_kinematics_node", options);
        kin_ = std::make_unique<rumblex_geometry::CBodyModel>(node_);
        cout << "KinematicsTest SetUp complete" << endl;
    }

    void TearDown() override {
        kin_.reset();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<rumblex_geometry::CBodyModel> kin_;
};

// Standing position
// RightFront: 	ag: 0.000°, 2.276°, 7.704°	| x: 0.201, y: -0.160, z: -0.050

// Laydown position
// RightFront: 	ag: 0.000°, 70.723°, -53.320°	| x: 0.180, y: -0.139, z: 0.010

TEST_F(KinematicsTest, setLegAngles) {
    // create target angles
    rumblex_geometry::CLegAngles angles;
    angles.torso_coxa = 0.0 * rumblex_geometry::units::deg;
    angles.coxa_femur = 2.276 * rumblex_geometry::units::deg;
    angles.femur_tibia = 7.704 * rumblex_geometry::units::deg;

    kin_->setLegAngles(rumblex_geometry::ELegIndex::RightFront, angles);

    rumblex_geometry::CPosition toe_pos_expected;
    toe_pos_expected.x = 0.201 * rumblex_geometry::units::m;   // 0.092 + 0.109
    toe_pos_expected.y = -0.160 * rumblex_geometry::units::m;  // -0.092 - 0.068
    toe_pos_expected.z = -0.050 * rumblex_geometry::units::m;  // 0.045 - 0.095

    auto& leg = kin_->getLegs().at(rumblex_geometry::ELegIndex::RightFront);

    expectPositionNear(toe_pos_expected, leg.toe_position, "setLegAngles position mismatch");
    expectAnglesNear(angles, leg.angles, "setLegAngles angles mismatch");
}

TEST_F(KinematicsTest, check_standing_position) {
    kin_->moveTorso(kin_->getStandingToePositions());
    auto& leg = kin_->getLegs().at(rumblex_geometry::ELegIndex::RightFront);

    rumblex_geometry::CPosition toe_pos_expected;
    toe_pos_expected.x = 0.201 * rumblex_geometry::units::m;   // 0.092 + 0.109
    toe_pos_expected.y = -0.160 * rumblex_geometry::units::m;  // -0.092 - 0.068
    toe_pos_expected.z = -0.050 * rumblex_geometry::units::m;  // 0.045 - 0.095

    rumblex_geometry::CLegAngles angles_expected;
    angles_expected.torso_coxa = 0.0 * rumblex_geometry::units::deg;
    angles_expected.coxa_femur = 2.276 * rumblex_geometry::units::deg;
    angles_expected.femur_tibia = 7.704 * rumblex_geometry::units::deg;

    expectPositionNear(toe_pos_expected, leg.toe_position, "standing position mismatch");
    expectAnglesNear(angles_expected, leg.angles, "standing angles mismatch");
}

TEST_F(KinematicsTest, check_laydown_position) {
    kin_->moveTorso(kin_->getLaydownToePositions());
    auto& leg = kin_->getLegs().at(rumblex_geometry::ELegIndex::RightFront);

    rumblex_geometry::CPosition toe_pos_expected;
    toe_pos_expected.x = 0.180 * rumblex_geometry::units::m;   // 0.071 + 0.109
    toe_pos_expected.y = -0.139 * rumblex_geometry::units::m;  // -0.071 - 0.068
    toe_pos_expected.z = 0.010 * rumblex_geometry::units::m;   // 0.045 - 0.035

    rumblex_geometry::CLegAngles angles_expected;
    angles_expected.torso_coxa = 0.0 * rumblex_geometry::units::deg;
    angles_expected.coxa_femur = 70.723 * rumblex_geometry::units::deg;
    angles_expected.femur_tibia = -53.320 * rumblex_geometry::units::deg;

    expectPositionNear(toe_pos_expected, leg.toe_position, "laydown position mismatch");
    expectAnglesNear(angles_expected, leg.angles, "laydown angles mismatch");
}

TEST_F(KinematicsTest, check_set_toe) {
    rumblex_geometry::CPosition target_pos;
    target_pos.x = 0.201 * rumblex_geometry::units::m;   // 0.092 + 0.109
    target_pos.y = -0.160 * rumblex_geometry::units::m;  // -0.092 - 0.068
    target_pos.z = -0.050 * rumblex_geometry::units::m;  // 0.045 - 0.095

    kin_->setToePosition(rumblex_geometry::ELegIndex::RightFront, target_pos);
    auto& leg = kin_->getLegs().at(rumblex_geometry::ELegIndex::RightFront);

    rumblex_geometry::CLegAngles angles_expected;
    angles_expected.torso_coxa = 0.0 * rumblex_geometry::units::deg;
    angles_expected.coxa_femur = 2.276 * rumblex_geometry::units::deg;
    angles_expected.femur_tibia = 7.704 * rumblex_geometry::units::deg;

    expectPositionNear(target_pos, leg.toe_position, "setToe position mismatch");
    expectAnglesNear(angles_expected, leg.angles, "setToe angles mismatch");
}

TEST_F(KinematicsTest, TorsoRotationPreservesLegLocalTarget) {
    kin_->moveTorso(kin_->getStandingToePositions());
    const auto standing_angles = kin_->getLegAngles(rumblex_geometry::ELegIndex::RightFront);

    // Rotate the coxa mount by 90 degrees about each axis and keep the same
    // leg-local toe target: (92, -92, -50) mm relative to the mount.
    const std::array<std::pair<rumblex_geometry::COrientation, rumblex_geometry::CPosition>, 3> cases = {{
        {rumblex_geometry::COrientation(90.0 * rumblex_geometry::units::deg,
                                        0.0 * rumblex_geometry::units::rad,
                                        0.0 * rumblex_geometry::units::rad),
         rumblex_geometry::CPosition(201.0 * rumblex_geometry::units::mm, -92.0 * rumblex_geometry::units::mm,
                                     -118.0 * rumblex_geometry::units::mm)},
        {rumblex_geometry::COrientation(0.0 * rumblex_geometry::units::rad,
                                        90.0 * rumblex_geometry::units::deg,
                                        0.0 * rumblex_geometry::units::rad),
         rumblex_geometry::CPosition(92.0 * rumblex_geometry::units::mm, -160.0 * rumblex_geometry::units::mm,
                                     -159.0 * rumblex_geometry::units::mm)},
        {rumblex_geometry::COrientation(0.0 * rumblex_geometry::units::rad,
                                        0.0 * rumblex_geometry::units::rad,
                                        90.0 * rumblex_geometry::units::deg),
         rumblex_geometry::CPosition(160.0 * rumblex_geometry::units::mm, 17.0 * rumblex_geometry::units::mm,
                                     -50.0 * rumblex_geometry::units::mm)},
    }};

    for (const auto& [orientation, target] : cases) {
        kin_->moveTorso({{rumblex_geometry::ELegIndex::RightFront, target}},
                        rumblex_geometry::CPose(rumblex_geometry::CPosition(), orientation));
        expectAnglesNear(standing_angles, kin_->getLegAngles(rumblex_geometry::ELegIndex::RightFront),
                         "rotated mount changed the leg-local target");
    }
}
