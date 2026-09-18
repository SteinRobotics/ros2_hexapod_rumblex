/*******************************************************************************
 * Copyright (c) 2023 Christian Stein
 ******************************************************************************/

#include "handler/movement.hpp"

using namespace std::chrono_literals;
using namespace rumblex_interfaces::msg;

namespace brain {

CMovement::CMovement(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    callback_timer_ = std::make_unique<CCallbackTimer>();
    pub_cmd_movement_ = node_->create_publisher<MovementRequest>("cmd_movement", 10);
    pub_cmd_movement_update_ = node_->create_publisher<ContinuousMovementUpdate>("cmd_movement_update", 10);
}

void CMovement::publishMovementRequest() {
    current_request_.header.stamp = node_->get_clock()->now();
    pub_cmd_movement_->publish(current_request_);
}

void CMovement::publishContinuousUpdate() {
    current_continuous_update_.header.stamp = node_->get_clock()->now();
    pub_cmd_movement_update_->publish(current_continuous_update_);
}

void CMovement::run(std::shared_ptr<RequestMovementType> request) {
    setDone(false);
    current_request_.type = request->movementRequest.type;
    current_request_.name = request->movementRequest.name;
    current_request_.direction = request->movementRequest.direction;
    current_request_.duration_s = request->movementRequest.duration_s;
    publishMovementRequest();
    callback_timer_->waitSecondsNonBlocking(request->movementRequest.duration_s,
                                            std::bind(&CMovement::timerCallback, this));
}

void CMovement::run(std::shared_ptr<RequestSinglePose> request) {
    current_continuous_update_.body_pose = request->pose;
    publishContinuousUpdate();
}

void CMovement::run(std::shared_ptr<RequestHeadOrientation> request) {
    current_continuous_update_.head_orientation = request->orientation;
    publishContinuousUpdate();
}

void CMovement::run(std::shared_ptr<RequestVelocity> request) {
    current_continuous_update_.velocity = request->velocity;
    publishContinuousUpdate();
}

void CMovement::timerCallback() {
    // TODO better trigger callback to request_executor::execute
    setDone(true);
}

void CMovement::cancel() {
}

void CMovement::update() {
}

}  // namespace brain
