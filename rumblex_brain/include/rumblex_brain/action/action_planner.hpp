/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include <list>
#include <vector>

#include "handler/communication.hpp"
#include "handler/ihandler.hpp"
#include "handler/movement.hpp"
#include "handler/system.hpp"
#include "rclcpp/rclcpp.hpp"
#include "requester/irequester.hpp"

namespace brain {

class CActionPlanner {
   public:
    CActionPlanner(std::shared_ptr<rclcpp::Node> node);
    virtual ~CActionPlanner() = default;

    void setMovementCallback(std::function<void(const MovementRequest&)> callback) {
        handler_movement_->on_gait_changed = std::move(callback);
    }
    void update();
    using InvocationId = uint64_t;
    InvocationId nextInvocationId() {
        return ++last_invocation_id_;
    }
    void request(std::vector<std::shared_ptr<RequestBase>> requests_v, Prio prio, InvocationId id = 0);
    void execute(std::vector<std::shared_ptr<RequestBase>>& requests_v);
    void cancelRunningRequest();
    // Cached result from the last handler update, before dispatching the next
    // group. This is scheduling completion, not a motion-rest indication.
    bool requestGroupFinished();

   private:
    friend class ActionPlannerTestAccess;
    struct RequestGroup : std::vector<std::shared_ptr<RequestBase>> {
        RequestGroup() = default;
        RequestGroup(const std::vector<std::shared_ptr<RequestBase>>& requests, InvocationId invocation = 0)
            : std::vector<std::shared_ptr<RequestBase>>(requests), id(invocation) {
        }
        InvocationId id = 0;
    };
    void abortActiveSequence();
    InvocationId last_invocation_id_ = 0;
    std::shared_ptr<rclcpp::Node> node_;
    std::list<RequestGroup> requests_highest_prio_;
    std::list<RequestGroup> requests_high_prio_;
    std::list<RequestGroup> requests_normal_prio_;
    RequestGroup active_requests_;
    RequestGroup request_background_;
    Prio active_prio_ = Prio::Background;

    std::vector<std::shared_ptr<IHandler>> handlers_;
    bool request_group_finished_ = true;
    std::shared_ptr<CSystem> handler_system_;
    std::shared_ptr<CCommunication> handler_communication_;
    std::shared_ptr<CMovement> handler_movement_;
};

}  // namespace brain
