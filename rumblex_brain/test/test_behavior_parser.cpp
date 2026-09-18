/*******************************************************************************
 * Quick test for behavior parser with multiple behaviors
 ******************************************************************************/

#include <rclcpp/rclcpp.hpp>

#include <filesystem>

#include "rumblex_brain/parser/behavior_parser.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("behavior_parser_test");

    brain::CBehaviorParser parser(node);

    const std::string configPath =
        (std::filesystem::path(__FILE__).parent_path().parent_path() / "config" / "behaviors.json")
            .string();

    RCLCPP_INFO(node->get_logger(), "Parsing behaviors from: %s", configPath.c_str());

    if (parser.parseFile(configPath)) {
        const auto& behaviors = parser.getBehaviors();
        RCLCPP_INFO(node->get_logger(), "Successfully parsed %zu behaviors", behaviors.size());

        for (const auto& behavior : behaviors) {
            RCLCPP_INFO(node->get_logger(), "  Behavior: %s with %zu action groups", behavior.name.c_str(),
                        behavior.actionGroups.size());

            for (size_t i = 0; i < behavior.actionGroups.size(); ++i) {
                RCLCPP_INFO(node->get_logger(), "    Action group %zu: %zu requests", i,
                            behavior.actionGroups[i].size());
            }
        }

        // Test getBehavior by name
        auto watchBehavior = parser.getBehavior("WATCH");
        if (watchBehavior) {
            RCLCPP_INFO(node->get_logger(), "Found WATCH behavior with %zu action groups",
                        watchBehavior->get().actionGroups.size());
        }

        auto lookBehavior = parser.getBehavior("LOOK");
        if (lookBehavior) {
            RCLCPP_INFO(node->get_logger(), "Found LOOK behavior with %zu action groups",
                        lookBehavior->get().actionGroups.size());
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to parse behaviors");
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
