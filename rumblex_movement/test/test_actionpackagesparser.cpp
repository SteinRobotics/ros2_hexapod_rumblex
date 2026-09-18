#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "requester/actionpackagesparser.hpp"
#include "test_helpers.hpp"

using namespace std;
using namespace rumblex_movement;

class ActionPackagesParserTest : public ::testing::Test {
   protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        node_ = std::make_shared<rclcpp::Node>("test_actionpackagesparser_node");
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

TEST_F(ActionPackagesParserTest, legAngles_standingPosition) {
    auto positions = parser_->getFootPositions("footPositions_standing");
    EXPECT_EQ(positions.size(), 6);

    // Check one leg's position as an example
    auto it = positions.find(ELegIndex::RightFront);
    ASSERT_NE(it, positions.end());
    const CPosition& position = it->second;

    CPosition expectedFootPos;
    expectedFootPos.x = 0.092 + 0.109;  // CENTER_TO_COXA_X + STANDING_FOOT_POS_X
    expectedFootPos.y = 0.092 + 0.068;  // CENTER_TO_COXA_Y + STANDING_FOOT_POS_Y
    expectedFootPos.z = -0.050;         // STANDING_FOOT_POS_Z
    expectPositionNear(expectedFootPos, position, "Standing foot position mismatch");
}

TEST_F(ActionPackagesParserTest, STAND_UP_PackageLoadsCorrectly) {
    auto& requests = parser_->getRequests("STAND_UP");
    EXPECT_FALSE(requests.empty());
    EXPECT_EQ(requests.size(), 1);

    const auto& firstRequest = requests[0];
    EXPECT_TRUE(firstRequest.head.has_value());

    // The following checks depend on the YAML content for STAND_UP
    EXPECT_FALSE(firstRequest.body.has_value());
    EXPECT_TRUE(firstRequest.legAngles.has_value());
    EXPECT_FALSE(firstRequest.footPositions.has_value());

    EXPECT_DOUBLE_EQ(firstRequest.factorDuration, 1.0);

    const auto& head = firstRequest.head.value();
    EXPECT_DOUBLE_EQ(head.yaw_deg, 0.0);
    EXPECT_DOUBLE_EQ(head.pitch_deg, 0.0);

    const auto& legAngles = firstRequest.legAngles.value();
    // helper signature: expectAnglesNear(expected, actual)
    expectAnglesNear(CLegAngles(0.0, 2.276, 7.704), legAngles.at(ELegIndex::RightFront));
    expectAnglesNear(CLegAngles(0.0, 2.276, 7.704), legAngles.at(ELegIndex::RightBack));
    expectAnglesNear(CLegAngles(0.0, 2.276, 7.704), legAngles.at(ELegIndex::RightMid));
    expectAnglesNear(CLegAngles(0.0, 2.276, 7.704), legAngles.at(ELegIndex::LeftFront));
    expectAnglesNear(CLegAngles(0.0, 2.276, 7.704), legAngles.at(ELegIndex::LeftMid));
    expectAnglesNear(CLegAngles(0.0, 2.276, 7.704), legAngles.at(ELegIndex::LeftBack));
}
TEST_F(ActionPackagesParserTest, LAYDOWN_PackagesLoadCorrectly) {
    auto& requests = parser_->getRequests("LAYDOWN");
    EXPECT_FALSE(requests.empty());
    EXPECT_EQ(requests.size(), 1);
    const auto& firstRequest = requests[0];
    EXPECT_TRUE(firstRequest.head.has_value());
    // YAML for LAYDOWN currently does not include an explicit body entry, so it should be empty
    EXPECT_FALSE(firstRequest.body.has_value());
    EXPECT_TRUE(firstRequest.legAngles.has_value());
    EXPECT_FALSE(firstRequest.footPositions.has_value());
    EXPECT_DOUBLE_EQ(firstRequest.factorDuration, 1.0);
    const auto& head = firstRequest.head.value();
    EXPECT_DOUBLE_EQ(head.yaw_deg, 0.0);
    EXPECT_DOUBLE_EQ(head.pitch_deg, -20.0);
    // no body to check for LAYDOWN
    const auto& legAngles = firstRequest.legAngles.value();
    expectAnglesNear(CLegAngles(0.0, 70.723, -53.320), legAngles.at(ELegIndex::RightFront));
    expectAnglesNear(CLegAngles(0.0, 70.723, -53.320), legAngles.at(ELegIndex::RightBack));
    expectAnglesNear(CLegAngles(0.0, 70.723, -53.320), legAngles.at(ELegIndex::RightMid));
    expectAnglesNear(CLegAngles(0.0, 70.723, -53.320), legAngles.at(ELegIndex::LeftFront));
    expectAnglesNear(CLegAngles(0.0, 70.723, -53.320), legAngles.at(ELegIndex::LeftMid));
    expectAnglesNear(CLegAngles(0.0, 70.723, -53.320), legAngles.at(ELegIndex::LeftBack));
}

TEST_F(ActionPackagesParserTest, HIGH_FIVE_PackagesLoadCorrectly) {
    auto& requests = parser_->getRequests("HIGH_FIVE");
    EXPECT_FALSE(requests.empty());
    EXPECT_EQ(requests.size(), 3);

    const auto& firstRequest = requests[0];
    EXPECT_TRUE(firstRequest.head.has_value());
    EXPECT_FALSE(firstRequest.body.has_value());
    EXPECT_TRUE(firstRequest.legAngles.has_value());
    EXPECT_FALSE(firstRequest.footPositions.has_value());
    EXPECT_DOUBLE_EQ(firstRequest.factorDuration, 0.33);
    const auto& head = firstRequest.head.value();
    EXPECT_DOUBLE_EQ(head.yaw_deg, 0.0);
    EXPECT_DOUBLE_EQ(head.pitch_deg, -20.0);
    const auto& legAngles = firstRequest.legAngles.value();
    expectAnglesNear(CLegAngles(20.0, 50.0, 60.0), legAngles.at(ELegIndex::RightFront));

    const auto& secondRequest = requests[1];
    EXPECT_FALSE(secondRequest.head.has_value());
    EXPECT_FALSE(secondRequest.body.has_value());
    EXPECT_FALSE(secondRequest.legAngles.has_value());
    EXPECT_FALSE(secondRequest.footPositions.has_value());
    EXPECT_DOUBLE_EQ(secondRequest.factorDuration, 0.33);

    const auto& thirdRequest = requests[2];
    EXPECT_TRUE(thirdRequest.head.has_value());
    EXPECT_FALSE(thirdRequest.body.has_value());
    EXPECT_TRUE(thirdRequest.legAngles.has_value());
    EXPECT_FALSE(thirdRequest.footPositions.has_value());
    EXPECT_DOUBLE_EQ(thirdRequest.factorDuration, 0.33);
    const auto& headThird = thirdRequest.head.value();
    EXPECT_DOUBLE_EQ(headThird.yaw_deg, 0.0);
    EXPECT_DOUBLE_EQ(headThird.pitch_deg, 0.0);
    const auto& legAnglesThird = thirdRequest.legAngles.value();
    expectAnglesNear(CLegAngles(0.0, 2.276, 7.704), legAnglesThird.at(ELegIndex::RightFront));
    expectAnglesNear(CLegAngles(0.0, 2.276, 7.704), legAnglesThird.at(ELegIndex::RightBack));
    expectAnglesNear(CLegAngles(0.0, 2.276, 7.704), legAnglesThird.at(ELegIndex::RightMid));
    expectAnglesNear(CLegAngles(0.0, 2.276, 7.704), legAnglesThird.at(ELegIndex::LeftFront));
    expectAnglesNear(CLegAngles(0.0, 2.276, 7.704), legAnglesThird.at(ELegIndex::LeftMid));
    expectAnglesNear(CLegAngles(0.0, 2.276, 7.704), legAnglesThird.at(ELegIndex::LeftBack));
}