/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "action/action_planner.hpp"

#include <utility>

namespace brain {
CActionPlanner::CActionPlanner(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    handler_system_ = std::make_shared<CSystem>(node);
    handler_communication_ = std::make_shared<CCommunication>(node);
    handler_movement_ = std::make_shared<CMovement>(node);

    handlers_.push_back(handler_system_);
    handlers_.push_back(handler_communication_);
    handlers_.push_back(handler_movement_);
}

void CActionPlanner::request(std::vector<std::shared_ptr<RequestBase>> requests, Prio prio) {
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "CActionPlanner:: new requests with prio " << std::to_underlying(prio));

    // Priority handling:
    // - Highest: Interrupts everything, clears all queues, cancels running requests, and executes immediately.
    // - High: If a Highest-priority request is active, ignore. If High is active, queue. If Normal is active, promote it to queue and execute High.
    // - Normal: If Highest is active, ignore. Otherwise, queue.
    // - Background: Only queued if nothing else is running.

    switch (prio) {
        case Prio::Highest:
            // Clear all other queues and background, cancel running request, and push to highest-priority queue
            requests_high_prio_.clear();
            requests_normal_prio_.clear();
            request_background_.clear();
            cancelRunningRequest();
            requests_highest_prio_.push_back(requests);
            break;

        case Prio::High:
            // If a Highest-priority request is active, ignore this one
            if (active_prio_ == Prio::Highest) break;
            // If a High-priority request is active, queue this one
            if (active_prio_ == Prio::High) {
                requests_high_prio_.push_back(requests);
                break;
            }
            // TODO I think better deactivate the following feature and remove the active_requests_
            // If a Normal-priority request is active, promote it to the normal queue and execute this High-priority request
            // if (active_prio_ == Prio::Normal) {
            //     requests_normal_prio_.push_back(active_requests_);
            // }
            cancelRunningRequest();
            requests_high_prio_.push_back(requests);
            break;

        case Prio::Normal:
            // If a Highest-priority request is active, ignore this one
            if (active_prio_ == Prio::Highest) break;
            // Otherwise, queue this normal-priority request
            requests_normal_prio_.push_back(requests);
            break;

        case Prio::Background:
            // If a Highest-priority request is active, ignore this one
            if (active_prio_ == Prio::Highest) break;
            // Set as background request (executed when nothing else is running)
            request_background_ = requests;
            break;

        default:
            RCLCPP_ERROR_STREAM(node_->get_logger(), "unknown prio of the new request");
            break;
    }
}

void CActionPlanner::update() {
    bool is_done = true;
    for (auto handler : handlers_) {
        handler->update();
        if (!handler->done()) {
            is_done = false;
        }
    }
    is_done_ = is_done;
    if (!is_done_) {
        return;
    }
    // RCLCPP_INFO_STREAM(node_->get_logger(), "schedule new request");
    if (!requests_highest_prio_.empty()) {
        active_requests_ = requests_highest_prio_.front();
        requests_highest_prio_.pop_front();
        active_prio_ = Prio::Highest;
    } else if (!requests_high_prio_.empty()) {
        active_requests_ = requests_high_prio_.front();
        requests_high_prio_.pop_front();
        active_prio_ = Prio::High;
    } else if (!requests_normal_prio_.empty()) {
        active_requests_ = requests_normal_prio_.front();
        requests_normal_prio_.pop_front();
        active_prio_ = Prio::Normal;
    } else {
        active_requests_ = request_background_;
        active_prio_ = Prio::Background;
    }
    execute(active_requests_);
}

/// here the old code from request_executor.cpp
void CActionPlanner::execute(std::vector<std::shared_ptr<RequestBase>>& requests_v) {
    // RCLCPP_INFO_STREAM(node_->get_logger(), "CActionPlanner::execute");
    for (const auto& request : requests_v) {
        if (auto requestSystem = std::dynamic_pointer_cast<RequestSystem>(request)) {
            handler_system_->run(requestSystem);
        } else if (auto requestTalking = std::dynamic_pointer_cast<RequestTalking>(request)) {
            handler_communication_->run(requestTalking);
        } else if (auto requestChat = std::dynamic_pointer_cast<RequestChat>(request)) {
            handler_communication_->run(requestChat);
        } else if (auto requestMusic = std::dynamic_pointer_cast<RequestMusic>(request)) {
            handler_communication_->run(requestMusic);
        } else if (auto requestListening = std::dynamic_pointer_cast<RequestListening>(request)) {
            handler_communication_->run(requestListening);
        } else if (auto requestMovementType = std::dynamic_pointer_cast<RequestMovementType>(request)) {
            handler_movement_->run(requestMovementType);
        } else if (auto requestMoveBody = std::dynamic_pointer_cast<RequestSinglePose>(request)) {
            handler_movement_->run(requestMoveBody);
        } else if (auto requestMoveVelocity = std::dynamic_pointer_cast<RequestVelocity>(request)) {
            handler_movement_->run(requestMoveVelocity);
        } else if (auto requestHeadOrientation = std::dynamic_pointer_cast<RequestHeadOrientation>(request)) {
            handler_movement_->run(requestHeadOrientation);
        } else {
            RCLCPP_ERROR_STREAM(node_->get_logger(),
                                "CActionPlanner: RequestType unknown: " << typeid(*request).name());
        }
    }
}

void CActionPlanner::cancelRunningRequest() {
    RCLCPP_INFO_STREAM(node_->get_logger(), "CActionPlanner::cancelRunningRequest");
    for (auto handler : handlers_) {
        handler->cancel();
    }
}

bool CActionPlanner::done() {
    return is_done_;
}

}  // namespace brain
