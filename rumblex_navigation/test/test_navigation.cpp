#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#define RUMBLEX_NAVIGATION_NO_MAIN
#include "../src/node_navigation.cpp"

using namespace std::chrono_literals;

TEST(NavigationOdometry, RequiresFeedbackAndDoesNotIntegrateCommands) {
    rclcpp::init(0, nullptr);
    auto navigation = std::make_shared<NodeNavigation>();
    auto test = std::make_shared<rclcpp::Node>("navigation_odometry_test");
    auto goals = test->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
    auto odometry = test->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    geometry_msgs::msg::Twist command;
    size_t samples = 0;
    auto commands = test->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, [&](const geometry_msgs::msg::Twist& message) {
            command = message;
            ++samples;
        });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(navigation);
    executor.add_node(test);
    auto pump = [&](std::chrono::milliseconds duration) {
        const auto end = std::chrono::steady_clock::now() + duration;
        do {
            executor.spin_some();
            std::this_thread::sleep_for(5ms);
        } while (std::chrono::steady_clock::now() < end);
        executor.spin_some();
    };
    pump(200ms);
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "odom";
    goal.pose.position.x = 0.5;
    goal.pose.orientation.w = 1.0;
    goals->publish(goal);
    pump(150ms);
    EXPECT_GT(samples, 0u);
    EXPECT_DOUBLE_EQ(command.linear.x, 0.0);

    nav_msgs::msg::Odometry pose;
    pose.header.frame_id = "odom";
    pose.child_frame_id = "base_link";
    pose.pose.pose.orientation.w = 1.0;
    // Repeated stationary feedback must keep requesting movement toward the goal.
    // Claimed fast command velocities cannot advance the navigation pose.
    pose.twist.twist.linear.x = 10.0;
    for (int i = 0; i < 5; ++i) {
        odometry->publish(pose);
        pump(120ms);
        EXPECT_GT(command.linear.x, 0.0);
    }
    pump(650ms);
    EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
    pose.pose.pose.position.x = 0.5;
    odometry->publish(pose);
    pump(150ms);
    EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
    // Arriving at the reported position ends the goal.
    pose.pose.pose.position.x = 0.0;
    odometry->publish(pose);
    pump(150ms);
    EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
    rclcpp::shutdown();
}
