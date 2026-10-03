/*******************************************************************************
 * Copyright (c) 2023 Christian Stein
 ******************************************************************************/

#include "action/action_planner.hpp"
#include "rclcpp/rclcpp.hpp"
#include "requester/coordinator.hpp"
#include "requester/recognition.hpp"

constexpr int REFRESH_RATE_HZ = 20;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("node_brain");

    auto actionPlanner = std::make_shared<brain::CActionPlanner>(node);
    auto coordinator = std::make_shared<brain::CCoordinator>(node, actionPlanner);
    actionPlanner->setMovementCallback(
        [weak = std::weak_ptr<brain::CCoordinator>(coordinator)](const brain::MovementRequest& request) {
            if (auto coordinator = weak.lock()) coordinator->movementTypeActualReceived(request);
        });
    auto recognition = std::make_shared<brain::CRecognition>(node, coordinator);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    rclcpp::Rate loop_rate(REFRESH_RATE_HZ);
    while (rclcpp::ok()) {
        coordinator->update();
        actionPlanner->update();
        executor.spin_some();
        loop_rate.sleep();
    }
    return 0;
}
