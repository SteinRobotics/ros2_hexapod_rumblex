#pragma once

#include <chrono>
#include <functional>
#include <optional>

#include "gait/gait_controller.hpp"
#include "ihandler.hpp"
#include "requester/irequester.hpp"
#include "rumblex_utils/body_pose.hpp"
#include "std_msgs/msg/string.hpp"

namespace brain {
class CMovement : public IHandler {
   public:
    explicit CMovement(std::shared_ptr<rclcpp::Node> node);
    void update() override;
    void cancel() override;
    void run(std::shared_ptr<RequestMovementType> request);
    void run(std::shared_ptr<RequestSinglePose> request);
    void run(std::shared_ptr<RequestHeadOrientation> request);
    void run(std::shared_ptr<RequestVelocity> request);
    std::function<void(const MovementRequest&)> on_gait_changed;

   private:
    void onInitialPose(const rumblex_interfaces::msg::BodyPose& pose);
    void startRequest(const MovementRequest& request);
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CPoseModel> pose_model_;
    std::unique_ptr<CGaitController> gait_controller_;
    rclcpp::Publisher<rumblex_interfaces::msg::BodyPose>::SharedPtr pub_body_pose_;
    rclcpp::Subscription<rumblex_interfaces::msg::BodyPose>::SharedPtr sub_body_pose_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_movement_name_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_movement_velocity_;
    geometry_msgs::msg::Twist velocity_;
    CPose torso_;
    COrientation head_;
    bool initialized_ = false;
    std::optional<MovementRequest> pending_request_;
    std::optional<rclcpp::Time> completion_time_;
    rumblex_interfaces::msg::BodyPose last_pose_;
    std::chrono::steady_clock::time_point next_update_{};
};
}  // namespace brain
