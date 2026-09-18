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

    void update();
    void request(std::vector<std::shared_ptr<RequestBase>> requests_v, Prio prio);
    void execute(std::vector<std::shared_ptr<RequestBase>>& requests_v);
    void cancelRunningRequest();
    bool done();

   private:
    std::shared_ptr<rclcpp::Node> node_;
    std::list<std::vector<std::shared_ptr<RequestBase>>> requests_highest_prio_;
    std::list<std::vector<std::shared_ptr<RequestBase>>> requests_high_prio_;
    std::list<std::vector<std::shared_ptr<RequestBase>>> requests_normal_prio_;
    std::vector<std::shared_ptr<RequestBase>> active_requests_;
    std::vector<std::shared_ptr<RequestBase>> request_background_;
    Prio active_prio_ = Prio::Background;

    std::vector<std::shared_ptr<IHandler>> handlers_;
    bool is_done_ = true;
    std::shared_ptr<CSystem> handler_system_;
    std::shared_ptr<CCommunication> handler_communication_;
    std::shared_ptr<CMovement> handler_movement_;
};

}  // namespace brain
