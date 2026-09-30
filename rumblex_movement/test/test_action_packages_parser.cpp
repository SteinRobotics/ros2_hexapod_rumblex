#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "requester/action_packages_parser.hpp"
#include "test_helpers.hpp"

using namespace std;
using namespace rumblex_movement;

class ActionPackagesParserTest : public ::testing::Test {
   protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        node_ = std::make_shared<rclcpp::Node>("test_action_packages_parser_node");
        parser_ = std::make_unique<CActionPackagesParser>(node_);
    }

    void TearDown() override {
        parser_.reset();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<CActionPackagesParser> parser_;
};

TEST_F(ActionPackagesParserTest, NonexistentPackageReturnsEmpty) {
    auto& requests = parser_->getRequests("this_package_does_not_exist_12345");
    EXPECT_TRUE(requests.empty());
}

TEST_F(ActionPackagesParserTest, leg_angles_standing_position) {
    auto positions = parser_->getToePositions("toe_positions_standing");
    EXPECT_EQ(positions.size(), 6);

    // Check one leg's position as an example
    auto it = positions.find(ELegIndex::RightFront);
    ASSERT_NE(it, positions.end());
    const CPosition& position = it->second;

    CPosition expected_toe_pos;
    expected_toe_pos.x = 0.092 + 0.109;      // CENTER_TO_COXA_X + STANDING_TOE_POS_X
    expected_toe_pos.y = 0.092 + 0.068;      // CENTER_TO_COXA_Y + STANDING_TOE_POS_Y
    expected_toe_pos.z = -0.050 * units::m;  // STANDING_TOE_POS_Z
    expectPositionNear(expected_toe_pos, position, "Standing toe position mismatch");
}

TEST_F(ActionPackagesParserTest, STAND_UP_PackageLoadsCorrectly) {
    auto& requests = parser_->getRequests("STAND_UP");
    EXPECT_FALSE(requests.empty());
    EXPECT_EQ(requests.size(), 1);

    const auto& first_request = requests[0];
    EXPECT_TRUE(first_request.head.has_value());

    // The following checks depend on the YAML content for STAND_UP
    EXPECT_FALSE(first_request.torso.has_value());
    EXPECT_TRUE(first_request.leg_angles.has_value());
    EXPECT_FALSE(first_request.toe_positions.has_value());

    EXPECT_DOUBLE_EQ(first_request.duration_factor, 1.0);

    const auto& head = first_request.head.value();
    EXPECT_DOUBLE_EQ(head.yaw.numerical_value_in(units::deg), 0.0);
    EXPECT_DOUBLE_EQ(head.pitch.numerical_value_in(units::deg), 0.0);

    const auto& leg_angles = first_request.leg_angles.value();
    // helper signature: expectAnglesNear(expected, actual)
    expectAnglesNear(CLegAngles(0.0, 2.276, 7.704), leg_angles.at(ELegIndex::RightFront));
    expectAnglesNear(CLegAngles(0.0, 2.276, 7.704), leg_angles.at(ELegIndex::RightBack));
    expectAnglesNear(CLegAngles(0.0, 2.276, 7.704), leg_angles.at(ELegIndex::RightMid));
    expectAnglesNear(CLegAngles(0.0, 2.276, 7.704), leg_angles.at(ELegIndex::LeftFront));
    expectAnglesNear(CLegAngles(0.0, 2.276, 7.704), leg_angles.at(ELegIndex::LeftMid));
    expectAnglesNear(CLegAngles(0.0, 2.276, 7.704), leg_angles.at(ELegIndex::LeftBack));
}
TEST_F(ActionPackagesParserTest, LAYDOWN_PackagesLoadCorrectly) {
    auto& requests = parser_->getRequests("LAYDOWN");
    EXPECT_FALSE(requests.empty());
    EXPECT_EQ(requests.size(), 1);
    const auto& first_request = requests[0];
    EXPECT_TRUE(first_request.head.has_value());
    // YAML for LAYDOWN currently does not include an explicit body entry, so it should be empty
    EXPECT_FALSE(first_request.torso.has_value());
    EXPECT_TRUE(first_request.leg_angles.has_value());
    EXPECT_FALSE(first_request.toe_positions.has_value());
    EXPECT_DOUBLE_EQ(first_request.duration_factor, 1.0);
    const auto& head = first_request.head.value();
    EXPECT_DOUBLE_EQ(head.yaw.numerical_value_in(units::deg), 0.0);
    EXPECT_DOUBLE_EQ(head.pitch.numerical_value_in(units::deg), -20.0);
    // no torso to check for LAYDOWN
    const auto& leg_angles = first_request.leg_angles.value();
    expectAnglesNear(CLegAngles(0.0, 70.723, -53.320), leg_angles.at(ELegIndex::RightFront));
    expectAnglesNear(CLegAngles(0.0, 70.723, -53.320), leg_angles.at(ELegIndex::RightBack));
    expectAnglesNear(CLegAngles(0.0, 70.723, -53.320), leg_angles.at(ELegIndex::RightMid));
    expectAnglesNear(CLegAngles(0.0, 70.723, -53.320), leg_angles.at(ELegIndex::LeftFront));
    expectAnglesNear(CLegAngles(0.0, 70.723, -53.320), leg_angles.at(ELegIndex::LeftMid));
    expectAnglesNear(CLegAngles(0.0, 70.723, -53.320), leg_angles.at(ELegIndex::LeftBack));
}

TEST_F(ActionPackagesParserTest, HIGH_FIVE_PackagesLoadCorrectly) {
    auto& requests = parser_->getRequests("HIGH_FIVE");
    EXPECT_FALSE(requests.empty());
    EXPECT_EQ(requests.size(), 3);

    const auto& first_request = requests[0];
    EXPECT_TRUE(first_request.head.has_value());
    EXPECT_FALSE(first_request.torso.has_value());
    EXPECT_TRUE(first_request.leg_angles.has_value());
    EXPECT_FALSE(first_request.toe_positions.has_value());
    EXPECT_DOUBLE_EQ(first_request.duration_factor, 0.33);
    const auto& head = first_request.head.value();
    EXPECT_DOUBLE_EQ(head.yaw.numerical_value_in(units::deg), 0.0);
    EXPECT_DOUBLE_EQ(head.pitch.numerical_value_in(units::deg), -20.0);
    const auto& leg_angles = first_request.leg_angles.value();
    expectAnglesNear(CLegAngles(20.0, 50.0, 60.0), leg_angles.at(ELegIndex::RightFront));

    const auto& second_request = requests[1];
    EXPECT_FALSE(second_request.head.has_value());
    EXPECT_FALSE(second_request.torso.has_value());
    EXPECT_FALSE(second_request.leg_angles.has_value());
    EXPECT_FALSE(second_request.toe_positions.has_value());
    EXPECT_DOUBLE_EQ(second_request.duration_factor, 0.33);

    const auto& third_request = requests[2];
    EXPECT_TRUE(third_request.head.has_value());
    EXPECT_FALSE(third_request.torso.has_value());
    EXPECT_TRUE(third_request.leg_angles.has_value());
    EXPECT_FALSE(third_request.toe_positions.has_value());
    EXPECT_DOUBLE_EQ(third_request.duration_factor, 0.33);
    const auto& head_third = third_request.head.value();
    EXPECT_DOUBLE_EQ(head_third.yaw.numerical_value_in(units::deg), 0.0);
    EXPECT_DOUBLE_EQ(head_third.pitch.numerical_value_in(units::deg), 0.0);
    const auto& leg_angles_third = third_request.leg_angles.value();
    expectAnglesNear(CLegAngles(0.0, 2.276, 7.704), leg_angles_third.at(ELegIndex::RightFront));
    expectAnglesNear(CLegAngles(0.0, 2.276, 7.704), leg_angles_third.at(ELegIndex::RightBack));
    expectAnglesNear(CLegAngles(0.0, 2.276, 7.704), leg_angles_third.at(ELegIndex::RightMid));
    expectAnglesNear(CLegAngles(0.0, 2.276, 7.704), leg_angles_third.at(ELegIndex::LeftFront));
    expectAnglesNear(CLegAngles(0.0, 2.276, 7.704), leg_angles_third.at(ELegIndex::LeftMid));
    expectAnglesNear(CLegAngles(0.0, 2.276, 7.704), leg_angles_third.at(ELegIndex::LeftBack));
}