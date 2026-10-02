#include <gtest/gtest.h>

#include <filesystem>

#include "requester/coordinator.hpp"

class BrainConfigurationTest : public ::testing::TestWithParam<std::string> {
   protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
    }
    void TearDown() override {
        rclcpp::shutdown();
    }
};
TEST_P(BrainConfigurationTest, LoadsRobotAnatomyAndGaits) {
    const auto root = std::filesystem::path(RUMBLEX_SOURCE_DIR);
    const auto brain = root / "rumblex_brain" / "config" / GetParam();
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--params-file", (brain / "parameter.yaml").string(), "--params-file",
                       (brain / "gait.yaml").string(), "--params-file",
                       (root / "rumblex_description" / "config" / GetParam() / "anatomy.yaml").string()});
    auto node = std::make_shared<rclcpp::Node>("node_brain", options);
    auto planner = std::make_shared<brain::CActionPlanner>(node);
    brain::CCoordinator coordinator(node, planner);
    EXPECT_GT(node->get_parameter("coxa_length_m").as_double(), 0.0);
    EXPECT_GT(node->get_parameter("gait.generic.torso_max_roll_deg").as_double(), 0.0);
    EXPECT_GT(node->get_parameter("gait.test_legs.coxa_femur_delta_deg").as_double(), 0.0);
    EXPECT_NO_THROW(coordinator.update());
    EXPECT_NO_THROW(planner->update());
}
INSTANTIATE_TEST_SUITE_P(Robots, BrainConfigurationTest, ::testing::Values("nox", "nira"));
