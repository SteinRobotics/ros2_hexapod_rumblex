/*******************************************************************************
 * Copyright (c) 2023 Christian Stein
 ******************************************************************************/

#pragma once

#include "ihandler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "requester/irequester.hpp"
#include "std_msgs/msg/bool.hpp"

namespace brain {

class CSystem : public IHandler {
   public:
    CSystem(std::shared_ptr<rclcpp::Node> node);
    virtual ~CSystem() = default;

    void update() override;
    void cancel() override;

    void run(std::shared_ptr<RequestSystem> request);

   private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubServoRelay_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubSystemShutdown_;
};

}  // namespace brain
