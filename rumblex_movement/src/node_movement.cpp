/*******************************************************************************
 * Copyright (c) 2023 Christian Stein
 ******************************************************************************/

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
//
#include "requester/requester.hpp"

constexpr double REFRESH_RATE_HZ = 10;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("node_movement");
    auto requester = std::make_shared<rumblex_movement::CRequester>(node);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    rclcpp::Rate loop_rate(REFRESH_RATE_HZ);
    auto timeslice_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / REFRESH_RATE_HZ));

    while (rclcpp::ok()) {
        executor.spin_some();
        requester->update(timeslice_ms);
        loop_rate.sleep();
    }
    return 0;
}
