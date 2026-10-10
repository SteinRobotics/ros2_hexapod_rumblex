#include <gtest/gtest.h>

#include <thread>

#include "handler/communication.hpp"

using namespace brain;
using namespace std::chrono_literals;

class CommunicationHandlerTest : public ::testing::Test {
   protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node = std::make_shared<rclcpp::Node>("communication_handler_test");
    }

    void TearDown() override {
        node.reset();
        rclcpp::shutdown();
    }

    std::shared_ptr<rclcpp::Node> node;
};

TEST_F(CommunicationHandlerTest, AcknowledgedListeningWaitsForOffPastTimeout) {
    CCommunication handler(node);
    auto request = std::make_shared<RequestListening>();
    request->active = true;
    handler.run(request);

    rumblex_interfaces::msg::CommunicationStatus status;
    status.status = rumblex_interfaces::msg::CommunicationStatus::STT_OFFLINE_ACTIVE;
    handler.onCommunicationStatus(status);
    status.status = rumblex_interfaces::msg::CommunicationStatus::STT_ONLINE_ACTIVE;
    handler.onCommunicationStatus(status);

    std::this_thread::sleep_for(5100ms);
    handler.update();
    EXPECT_FALSE(handler.done());

    status.status = rumblex_interfaces::msg::CommunicationStatus::OFF;
    handler.onCommunicationStatus(status);
    EXPECT_TRUE(handler.done());
}

TEST_F(CommunicationHandlerTest, NewRequestRequiresFreshAcknowledgment) {
    CCommunication handler(node);
    auto request = std::make_shared<RequestListening>();
    request->active = true;
    handler.run(request);

    rumblex_interfaces::msg::CommunicationStatus status;
    status.status = rumblex_interfaces::msg::CommunicationStatus::OFF;
    handler.onCommunicationStatus(status);
    ASSERT_TRUE(handler.done());

    handler.run(request);
    handler.update();
    ASSERT_FALSE(handler.done());

    std::this_thread::sleep_for(5100ms);
    handler.update();
    EXPECT_TRUE(handler.done());
}
