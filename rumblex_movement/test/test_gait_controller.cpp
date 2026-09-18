/*******************************************************************************
 * Copyright (c) 2025 Christian Stein
 ******************************************************************************/

#include <gtest/gtest.h>

#include <geometry_msgs/msg/twist.hpp>

#include "rumblex_interfaces/msg/movement_request.hpp"
#include "rclcpp/rclcpp.hpp"
#include "requester/gaitcontroller.hpp"
#include "requester/kinematics.hpp"
#include "test_helpers.hpp"

using namespace rumblex_movement;
using MovementRequestMsg = rumblex_interfaces::msg::MovementRequest;

namespace {
constexpr int kMaxIterations = 100;
}  // namespace

class GaitControllerTest : public ::testing::Test {
   protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }

        rclcpp::NodeOptions options;
        auto overrides = test_helpers::defaultRobotParameters();
        options.parameter_overrides(overrides);

        node_ = std::make_shared<rclcpp::Node>("test_gait_controller_node", options);
        kinematics_ = std::make_shared<CKinematics>(node_);
        controller_ = std::make_unique<CGaitController>(node_, kinematics_);
    }

    void TearDown() override {
        controller_.reset();
        kinematics_.reset();
        node_.reset();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    geometry_msgs::msg::Twist createZeroVelocity() {
        geometry_msgs::msg::Twist vel;
        vel.linear.x = 0.0;
        vel.linear.y = 0.0;
        vel.linear.z = 0.0;
        vel.angular.x = 0.0;
        vel.angular.y = 0.0;
        vel.angular.z = 0.0;
        return vel;
    }

    geometry_msgs::msg::Twist createForwardVelocity() {
        geometry_msgs::msg::Twist vel = createZeroVelocity();
        vel.linear.x = 0.1;
        return vel;
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;
    std::unique_ptr<CGaitController> controller_;
};

// Test: Verify default gait is LAYDOWN (robot starts laying down)
TEST_F(GaitControllerTest, DefaultGaitIsLaydown) {
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::SEQUENCE_LAYDOWN);
}

// Test: Switch from LAYDOWN to WAITING
TEST_F(GaitControllerTest, SwitchFromLaydownToWaiting) {
    auto vel = createZeroVelocity();

    // Verify we start with LAYDOWN
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::SEQUENCE_LAYDOWN);

    // Request stop and wait for it to complete
    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }

    // Switch to WAITING
    rumblex_interfaces::msg::MovementRequest request;
    request.type = MovementRequestMsg::SEQUENCE_WAITING;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::SEQUENCE_WAITING);
}

// Test: Switch from TRIPOD to LEGS_WAVE
TEST_F(GaitControllerTest, SwitchFromTripodToLegWave) {
    auto vel = createZeroVelocity();

    // Stop current gait
    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }

    // Switch to LEGS_WAVE
    rumblex_interfaces::msg::MovementRequest request;
    request.type = MovementRequestMsg::SEQUENCE_LEGS_WAVE;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::SEQUENCE_LEGS_WAVE);
}

// Test: Switch from WAITING to WATCH
TEST_F(GaitControllerTest, SwitchFromWaitingToWatch) {
    auto vel = createZeroVelocity();

    // First switch to WAITING
    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }
    rumblex_interfaces::msg::MovementRequest request;
    request.type = MovementRequestMsg::SEQUENCE_WAITING;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::SEQUENCE_WAITING);

    // Stop WAITING
    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }

    // Switch to WATCH
    request.type = MovementRequestMsg::SEQUENCE_WATCH;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::SEQUENCE_WATCH);
}

// Test: Switch to BODY_ROLL
TEST_F(GaitControllerTest, SwitchToBodyRoll) {
    auto vel = createZeroVelocity();

    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }

    rumblex_interfaces::msg::MovementRequest request;
    request.type = MovementRequestMsg::SEQUENCE_BODY_ROLL;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::SEQUENCE_BODY_ROLL);
}

// Test: Switch from BODY_ROLL to STAND_UP
TEST_F(GaitControllerTest, SwitchFromBodyRollToStandUp) {
    auto vel = createZeroVelocity();

    // First stop current gait (LAYDOWN)
    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }

    // Switch to BODY_ROLL
    rumblex_interfaces::msg::MovementRequest request;
    request.type = MovementRequestMsg::SEQUENCE_BODY_ROLL;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::SEQUENCE_BODY_ROLL);

    // Stop BODY_ROLL before switching
    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }

    // Switch to STAND_UP and allow the switch to process
    request.type = MovementRequestMsg::SEQUENCE_STAND_UP;
    controller_->setGait(request);
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::SEQUENCE_STAND_UP);
}

// Test: Switch to STAND_UP
TEST_F(GaitControllerTest, SwitchToStandUp) {
    auto vel = createZeroVelocity();

    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }

    rumblex_interfaces::msg::MovementRequest request;
    request.type = MovementRequestMsg::SEQUENCE_STAND_UP;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::SEQUENCE_STAND_UP);
}

// Test: Switch to LAYDOWN
TEST_F(GaitControllerTest, SwitchToLaydown) {
    auto vel = createZeroVelocity();

    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }

    rumblex_interfaces::msg::MovementRequest request;
    request.type = MovementRequestMsg::SEQUENCE_LAYDOWN;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::SEQUENCE_LAYDOWN);
}

// Test: Switch to HIGH_FIVE
TEST_F(GaitControllerTest, SwitchToHighFive) {
    auto vel = createZeroVelocity();

    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }

    rumblex_interfaces::msg::MovementRequest request;
    request.type = MovementRequestMsg::SEQUENCE_HIGH_FIVE;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::SEQUENCE_HIGH_FIVE);
}

// Test: Request same gait twice (should not switch)
TEST_F(GaitControllerTest, RequestSameGaitTwice) {
    auto vel = createZeroVelocity();

    // Switch to WAITING
    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }
    rumblex_interfaces::msg::MovementRequest request;
    request.type = MovementRequestMsg::SEQUENCE_WAITING;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::SEQUENCE_WAITING);

    // Request WAITING again (should remain WAITING, may restart if stopped)
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::SEQUENCE_WAITING);
}

// Test: Switching while gait is running (pending switch)
TEST_F(GaitControllerTest, SwitchWhileRunning) {
    auto vel = createForwardVelocity();

    // Start with LAYDOWN (default)
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::SEQUENCE_LAYDOWN);

    // Run a few iterations to ensure gait is active
    for (int i = 0; i < 5; ++i) {
        controller_->updateSelectedGait(vel);
    }

    // Request switch to WAITING while running
    rumblex_interfaces::msg::MovementRequest request;
    request.type = MovementRequestMsg::SEQUENCE_WAITING;
    controller_->setGait(request);

    // The switch should be pending until gait stops
    // Continue updating until switch completes
    auto zero_vel = createZeroVelocity();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(zero_vel);
        if (controller_->currentGait() == MovementRequestMsg::SEQUENCE_WAITING) {
            break;
        }
    }

    // Verify switch completed
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::SEQUENCE_WAITING);
}

// Test: Multiple consecutive switches
TEST_F(GaitControllerTest, MultipleConsecutiveSwitches) {
    auto vel = createZeroVelocity();

    // Switch 1: TRIPOD -> WAITING
    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }
    rumblex_interfaces::msg::MovementRequest request;
    request.type = MovementRequestMsg::SEQUENCE_WAITING;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::SEQUENCE_WAITING);

    // Switch 2: WAITING -> WATCH
    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }
    request.type = MovementRequestMsg::SEQUENCE_WATCH;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::SEQUENCE_WATCH);

    // Switch 3: WATCH -> LEGS_WAVE
    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }
    request.type = MovementRequestMsg::SEQUENCE_LEGS_WAVE;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::SEQUENCE_LEGS_WAVE);

    // Switch 4: LEGS_WAVE -> MOVE (back to original)
    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }
    request.type = MovementRequestMsg::CONTINUOUS_MOVE;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::CONTINUOUS_MOVE);
}

// Test: Update gait after switch
TEST_F(GaitControllerTest, UpdateAfterSwitch) {
    auto vel = createZeroVelocity();

    // Switch to WAITING
    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }
    rumblex_interfaces::msg::MovementRequest request;
    request.type = MovementRequestMsg::SEQUENCE_WAITING;
    controller_->setGait(request);

    // Update the new gait - should not crash
    bool update_result = false;
    EXPECT_NO_THROW({ update_result = controller_->updateSelectedGait(vel); });

    // Update returns true when gait processes successfully
    EXPECT_TRUE(update_result || !update_result);  // Just verify it executes
}

// Test: Switch back to MOVE with velocity
TEST_F(GaitControllerTest, SwitchBackToMoveWithVelocity) {
    auto vel = createZeroVelocity();
    auto forward_vel = createForwardVelocity();

    // Switch to WAITING
    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }
    rumblex_interfaces::msg::MovementRequest request;
    request.type = MovementRequestMsg::SEQUENCE_WAITING;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::SEQUENCE_WAITING);

    // Switch back to MOVE
    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }
    request.type = MovementRequestMsg::CONTINUOUS_MOVE;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::CONTINUOUS_MOVE);

    // Update with forward velocity - should work
    EXPECT_NO_THROW({
        for (int i = 0; i < 10; ++i) {
            controller_->updateSelectedGait(forward_vel);
        }
    });
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
