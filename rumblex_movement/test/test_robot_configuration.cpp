#include <gtest/gtest.h>

#include <filesystem>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "requester/requester.hpp"

using namespace rumblex_movement;

class RobotConfigurationTest : public ::testing::TestWithParam<std::string> {
   protected:
    void SetUp() override {
        if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    }

    void TearDown() override {
        if (rclcpp::ok()) rclcpp::shutdown();
    }
};

TEST_P(RobotConfigurationTest, StartsOfflineWithRobotYaml) {
    const auto config_dir = std::filesystem::path(RUMBLEX_CONFIG_DIR) / GetParam();
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--params-file", (config_dir / "anatomy.yaml").string(), "--params-file",
                       (config_dir / "servo_description.yaml").string()});
    options.parameter_overrides({rclcpp::Parameter("servo.offline", true)});
    auto node = std::make_shared<rclcpp::Node>("node_movement", options);

    CRequester requester(node);
    EXPECT_NO_THROW(requester.update(std::chrono::milliseconds(0)));
    EXPECT_EQ(node->get_parameter("leg_names.right_front").as_string(), "right_front");
    EXPECT_GT(node->get_parameter("toe_positions_standing.right_front.x").as_double(), 0.0);
    EXPECT_GT(node->get_parameter("gait.generic.torso_max_roll_deg").as_double(), 0.0);
    EXPECT_GT(node->get_parameter("gait.test_legs.coxa_femur_delta_deg").as_double(), 0.0);
    EXPECT_EQ(node->get_parameter("servo.names").as_string_array().size(), 20u);
}

INSTANTIATE_TEST_SUITE_P(Robots, RobotConfigurationTest, ::testing::Values("nox", "nira"));
