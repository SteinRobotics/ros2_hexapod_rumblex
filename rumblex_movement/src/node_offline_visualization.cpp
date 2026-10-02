#include "offline_visualization.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("offline_visualization");
    rumblex_geometry::CBodyModel model(node);
    auto publisher = node->create_publisher<sensor_msgs::msg::JointState>("visualization_joint_states", 10);
    auto subscription = node->create_subscription<rumblex_interfaces::msg::BodyPose>(
        "body_pose_actual", rclcpp::QoS(1).transient_local(),
        [&](const rumblex_interfaces::msg::BodyPose& pose) {
            if (!rumblex_geometry::validBodyPose(pose)) {
                RCLCPP_WARN(node->get_logger(), "Ignoring non-finite offline body pose");
                return;
            }
            auto joints = rumblex_movement::visualizationJoints(model, pose);
            if (!std::all_of(joints.position.begin(), joints.position.end(),
                             [](double angle) { return std::isfinite(angle); }))
                return;
            joints.header.stamp = node->now();
            publisher->publish(joints);
        });
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
