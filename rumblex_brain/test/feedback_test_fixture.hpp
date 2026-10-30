#pragma once

#include <gtest/gtest.h>

#include <thread>

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "test_helpers.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"

namespace {
using namespace brain;
class FeedbackTestFixture : public ::testing::Test {
   protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
    }
    void TearDown() override {
        executor.reset();
        broadcaster.reset();
        node.reset();
        rclcpp::shutdown();
    }
    void create(const std::vector<rclcpp::Parameter>& parameters) {
        rclcpp::NodeOptions options;
        options.parameter_overrides(parameters);
        node = std::make_shared<rclcpp::Node>("feedback_sensors_test", options);
        imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("test_imu", rclcpp::SensorDataQoS());
        range_pub = node->create_publisher<sensor_msgs::msg::Range>("test_range", rclcpp::SensorDataQoS());
        broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(
            tf2_ros::StaticTransformBroadcaster::RequiredInterfaces(*node));
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = "base_link";
        transform.child_frame_id = "rotated_sensor";
        transform.header.stamp = node->now();
        mount.setRPY(0, 0, -1.5707963267948966);  // Nira's upright measurement frame.
        transform.transform.rotation.x = mount.x();
        transform.transform.rotation.y = mount.y();
        transform.transform.rotation.z = mount.z();
        transform.transform.rotation.w = mount.w();
        broadcaster->sendTransform(transform);
        executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        executor->add_node(node);
        spin();
    }
    void spin() {
        for (int i = 0; i < 20; ++i) {
            executor->spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    void publishImu(double roll = 0.0, double pitch = 0.0) {
        tf2::Quaternion body;
        body.setRPY(roll, pitch, 0);
        const auto q = body * mount;
        imu.header.stamp = node->now();
        imu.header.frame_id = "rotated_sensor";
        imu.orientation.x = q.x();
        imu.orientation.y = q.y();
        imu.orientation.z = q.z();
        imu.orientation.w = q.w();
        imu_pub->publish(imu);
        spin();
    }
    void publishRange() {
        range.header.stamp = node->now();
        range.header.frame_id = "base_link";
        range.range = 0.8;
        range.min_range = 0.05;
        range.max_range = 10;
        range_pub->publish(range);
        spin();
    }
    std::shared_ptr<rclcpp::Node> node;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub;
    sensor_msgs::msg::Imu imu;
    sensor_msgs::msg::Range range;
    tf2::Quaternion mount;
};
}  // namespace
