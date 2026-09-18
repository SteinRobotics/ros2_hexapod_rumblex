/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "handler/communication.hpp"

// using namespace rumblex_communication;
using std::placeholders::_1;

namespace brain {

CCommunication::CCommunication(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    m_subStatus = node_->create_subscription<rumblex_interfaces::msg::CommunicationStatus>(
        "communication_status", 10, std::bind(&CCommunication::onCommunicationStatus, this, _1));

    m_pubTalking = node_->create_publisher<std_msgs::msg::String>("request_talking", 10);
    m_pubChat = node_->create_publisher<std_msgs::msg::String>("request_chat", 10);
    m_pubListening = node_->create_publisher<std_msgs::msg::Bool>("request_listening", 10);
    m_pubMusic = node_->create_publisher<std_msgs::msg::String>("request_music", 10);
    // timer = nh->createTimer(ros::Duration(0.1), &CCommunication::timerCallback, this);
}

void CCommunication::onCommunicationStatus(const rumblex_interfaces::msg::CommunicationStatus& msg) {
    auto status_string = std::to_string(msg.status);
    RCLCPP_INFO_STREAM(node_->get_logger(), "onCommunicationStatus |" << status_string.c_str() << "|");
    m_communication.status = msg.status;
    if (!done() && msg.status == rumblex_interfaces::msg::CommunicationStatus::OFF) {
        setDone(true);
    }
}

void CCommunication::run(std::shared_ptr<RequestListening> request) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "CCommunication::run Listening |" << request->active);
    std_msgs::msg::Bool msg;
    msg.data = request->active;
    m_pubListening->publish(msg);
    setDone(false);
    request_time_ = std::chrono::steady_clock::now();
}

void CCommunication::run(std::shared_ptr<RequestTalking> request) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "CCommunication::run Talking |" << request->text.c_str());
    std_msgs::msg::String msg;
    msg.data = request->text;
    m_pubTalking->publish(msg);
    setDone(false);
    request_time_ = std::chrono::steady_clock::now();
}

void CCommunication::run(std::shared_ptr<RequestChat> request) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "CCommunication::run Chat |" << request->text.c_str());
    std_msgs::msg::String msg;
    msg.data = request->text;
    m_pubChat->publish(msg);
    setDone(false);
    request_time_ = std::chrono::steady_clock::now();
}

void CCommunication::run(std::shared_ptr<RequestMusic> request) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "CCommunication::run Music |" << request->song.c_str());
    std_msgs::msg::String msg;
    msg.data = request->song;
    m_pubMusic->publish(msg);
    setDone(false);
    request_time_ = std::chrono::steady_clock::now();
}

void CCommunication::cancel() {
    if (m_communication.status == rumblex_interfaces::msg::CommunicationStatus::STT_OFFLINE_ACTIVE) {
        auto request = std::make_shared<RequestListening>();
        request->active = false;
        run(request);
    }
    if (m_communication.status == rumblex_interfaces::msg::CommunicationStatus::PLAYING_MUSIC) {
        auto request = std::make_shared<RequestMusic>();
        request->song = "STOP";
        run(request);
    }
}

void CCommunication::update() {
    if (!done() && std::chrono::steady_clock::now() - request_time_ > kTimeout) {
        RCLCPP_WARN(node_->get_logger(), "CCommunication: no status received within timeout, marking done");
        setDone(true);
    }
}

}  // namespace brain
