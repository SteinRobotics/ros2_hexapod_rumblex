/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include <chrono>

#include "ihandler.hpp"
#include "rumblex_interfaces/msg/communication_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "requester/irequester.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

namespace brain {

class CCommunication : public IHandler {
   public:
    CCommunication(std::shared_ptr<rclcpp::Node> node);
    virtual ~CCommunication() = default;

    void onCommunicationStatus(const rumblex_interfaces::msg::CommunicationStatus& msg);

    void update() override;
    void cancel() override;

    void run(std::shared_ptr<RequestListening> request);
    void run(std::shared_ptr<RequestTalking> request);
    void run(std::shared_ptr<RequestChat> request);
    void run(std::shared_ptr<RequestMusic> request);

   private:
    void timerCallback();

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pubTalking;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pubChat;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pubListening;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pubMusic;
    rclcpp::Subscription<rumblex_interfaces::msg::CommunicationStatus>::SharedPtr m_subStatus;

    rumblex_interfaces::msg::CommunicationStatus m_communication =
        rumblex_interfaces::msg::CommunicationStatus();

    static constexpr std::chrono::seconds kTimeout{5};
    std::chrono::steady_clock::time_point request_time_{};
};

}  // namespace brain