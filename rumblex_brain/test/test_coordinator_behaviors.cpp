/*******************************************************************************
 * Test for coordinator with behavior parser integration
 ******************************************************************************/

#include <rclcpp/rclcpp.hpp>

#include "action/action_planner.hpp"
#include "requester/coordinator.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_coordinator_behaviors");

    // Create minimal action planner for testing
    auto actionPlanner = std::make_shared<brain::CActionPlanner>(node);

    // Create coordinator (this will load behaviors)
    auto coordinator = std::make_shared<brain::CCoordinator>(node, actionPlanner);

    RCLCPP_INFO(node->get_logger(), "Coordinator created successfully with behavior parser");
    RCLCPP_INFO(node->get_logger(), "Test completed successfully");

    rclcpp::shutdown();
    return 0;
}
