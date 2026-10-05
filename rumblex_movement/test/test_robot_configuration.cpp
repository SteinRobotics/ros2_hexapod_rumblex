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
    options.arguments({"--ros-args", "--params-file",
                       (std::filesystem::path(RUMBLEX_ANATOMY_DIR) / GetParam() / "anatomy.yaml").string(),
                       "--params-file", (config_dir / "servo_description.yaml").string()});
    options.parameter_overrides({rclcpp::Parameter("servo.offline", true)});
    auto node = std::make_shared<rclcpp::Node>("node_movement", options);

    CRequester requester(node);
    EXPECT_NO_THROW(requester.update(std::chrono::milliseconds(0)));
    EXPECT_EQ(node->get_parameter("leg_names.right_front").as_string(), "right_front");
    EXPECT_GT(node->get_parameter("toe_positions_standing.right_front.x").as_double(), 0.0);
    for (const std::string side : {"right", "left"}) {
        const double sign = side == "right" ? -1.0 : 1.0;
        for (const std::string location : {"front", "mid", "back"}) {
            const auto leg = side + "_" + location;
            SCOPED_TRACE(leg);
            EXPECT_GT(sign * node->get_parameter("leg_offsets." + leg + ".y_m").as_double(), 0.0);
            EXPECT_GT(sign * node->get_parameter("leg_offsets." + leg + ".yaw_deg").as_double(), 0.0);
            for (const std::string pose : {"standing", "laydown"}) {
                EXPECT_GT(sign * node->get_parameter("toe_positions_" + pose + "." + leg + ".y").as_double(),
                          0.0);
            }
        }
    }
    EXPECT_EQ(node->get_parameter("servo.names").as_string_array().size(), 20u);
}

INSTANTIATE_TEST_SUITE_P(Robots, RobotConfigurationTest, ::testing::Values("nox", "nira"));
