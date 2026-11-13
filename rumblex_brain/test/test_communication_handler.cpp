#include <gtest/gtest.h>

#include <thread>

#include "action/action_planner.hpp"
#include "handler/communication.hpp"
#include "test_helpers.hpp"

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
    EXPECT_FALSE(handler.requestFinished());

    status.status = rumblex_interfaces::msg::CommunicationStatus::OFF;
    handler.onCommunicationStatus(status);
    EXPECT_TRUE(handler.requestFinished());
}

TEST_F(CommunicationHandlerTest, NewRequestRequiresFreshAcknowledgment) {
    CCommunication handler(node);
    auto request = std::make_shared<RequestListening>();
    request->active = true;
    handler.run(request);

    rumblex_interfaces::msg::CommunicationStatus status;
    status.status = rumblex_interfaces::msg::CommunicationStatus::OFF;
    handler.onCommunicationStatus(status);
    ASSERT_TRUE(handler.requestFinished());

    handler.run(request);
    handler.update();
    ASSERT_FALSE(handler.requestFinished());

    std::this_thread::sleep_for(5100ms);
    handler.update();
    EXPECT_TRUE(handler.requestFinished());
}

TEST_F(CommunicationHandlerTest, BackgroundMusicReleasesPlannerAndSurvivesCancellation) {
    CCommunication handler(node);
    auto request = std::make_shared<RequestMusic>();
    request->song = "musicfox_excess_party.mp3";
    request->background = true;
    handler.run(request);
    EXPECT_TRUE(handler.requestFinished());

    // An interruption can arrive before or after the playback status.
    handler.cancel();
    EXPECT_TRUE(handler.requestFinished());
    rumblex_interfaces::msg::CommunicationStatus status;
    status.status = rumblex_interfaces::msg::CommunicationStatus::PLAYING_MUSIC;
    handler.onCommunicationStatus(status);
    handler.update();
    EXPECT_TRUE(handler.requestFinished());
    handler.cancel();
    EXPECT_TRUE(handler.requestFinished());

    // Standup's explicit stop still runs and waits for acknowledgment.
    auto stop = std::make_shared<RequestMusic>();
    stop->song = "STOP";
    handler.run(stop);
    EXPECT_FALSE(handler.requestFinished());
    status.status = rumblex_interfaces::msg::CommunicationStatus::OFF;
    handler.onCommunicationStatus(status);
    EXPECT_TRUE(handler.requestFinished());
}

TEST_F(CommunicationHandlerTest, ForegroundMusicStillWaitsAndCanBeCancelled) {
    CCommunication handler(node);
    auto request = std::make_shared<RequestMusic>();
    request->song = "dance.mp3";
    handler.run(request);
    EXPECT_FALSE(handler.requestFinished());
    rumblex_interfaces::msg::CommunicationStatus status;
    status.status = rumblex_interfaces::msg::CommunicationStatus::PLAYING_MUSIC;
    handler.onCommunicationStatus(status);
    handler.cancel();
    EXPECT_FALSE(handler.requestFinished());
    status.status = rumblex_interfaces::msg::CommunicationStatus::OFF;
    handler.onCommunicationStatus(status);
    EXPECT_TRUE(handler.requestFinished());
}

TEST_F(CommunicationHandlerTest, BackgroundMusicDoesNotFinishPendingSpeech) {
    CCommunication handler(node);
    handler.run(std::make_shared<RequestTalking>());
    auto request = std::make_shared<RequestMusic>();
    request->song = "musicfox_excess_party.mp3";
    request->background = true;
    handler.run(request);
    EXPECT_FALSE(handler.requestFinished());
}

TEST_F(CommunicationHandlerTest, PlannerStartsNextSequenceWhileBackgroundMusicPlays) {
    rclcpp::NodeOptions options;
    options.parameter_overrides(test_helpers::defaultRobotParameters());
    auto planner_node = std::make_shared<rclcpp::Node>("music_planner_test", options);
    CActionPlanner planner(planner_node);
    auto music = std::make_shared<RequestMusic>();
    music->song = "musicfox_excess_party.mp3";
    music->background = true;
    planner.request({music}, Prio::Normal, planner.nextInvocationId());
    planner.update();

    auto speech = std::make_shared<RequestTalking>();
    speech->text = "Next sequence";
    planner.request({speech}, Prio::Normal, planner.nextInvocationId());
    planner.update();
    planner.update();
    // The next sequence has started and is waiting for its speech response.
    EXPECT_FALSE(planner.requestGroupFinished());
}

TEST_F(CommunicationHandlerTest, BackgroundFadePublishesDurationWithoutBlockingOrFinishingSpeech) {
    CCommunication handler(node);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    float received = 0;
    auto subscription = node->create_subscription<std_msgs::msg::Float32>(
        "request_background_music_fade", 10, [&](const std_msgs::msg::Float32& msg) { received = msg.data; });
    const auto discovery_deadline = std::chrono::steady_clock::now() + 2s;
    while (subscription->get_publisher_count() == 0 &&
           std::chrono::steady_clock::now() < discovery_deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(10ms);
    }
    ASSERT_GT(subscription->get_publisher_count(), 0u);
    auto fade = std::make_shared<RequestMusic>();
    fade->song = "STOP";
    fade->fade_out_s = 4.0f;
    handler.run(fade);
    EXPECT_TRUE(handler.requestFinished());
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (received == 0 && std::chrono::steady_clock::now() < deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(10ms);
    }
    EXPECT_FLOAT_EQ(received, 4.0f);
    handler.run(std::make_shared<RequestTalking>());
    handler.run(fade);
    EXPECT_FALSE(handler.requestFinished());
}
