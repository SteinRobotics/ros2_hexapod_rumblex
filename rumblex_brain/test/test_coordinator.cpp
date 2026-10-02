#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <stdexcept>

#include "requester/coordinator.hpp"
#include "test_helpers.hpp"

namespace brain {

// Exercise request timing without sleeps or exposing coordinator state to callers.
class CoordinatorTestAccess {
   public:
    static void submit(CCoordinator& coordinator, uint32_t type, double seconds) {
        coordinator.submitRequestMove(type, seconds);
    }
    static bool locked(const CCoordinator& coordinator) {
        return coordinator.movement_deadline_.has_value();
    }
    static uint32_t actualType(const CCoordinator& coordinator) {
        return coordinator.actualMovementType_;
    }
    static void expire(CCoordinator& coordinator) {
        coordinator.movement_deadline_->time = {};
    }
};

class CoordinatorTest : public ::testing::Test {
   protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
    }
    void TearDown() override {
        coordinator_.reset();
        planner_.reset();
        node_.reset();
        rclcpp::shutdown();
    }
    void create(std::vector<rclcpp::Parameter> parameters = test_helpers::defaultCoordinatorParameters()) {
        coordinator_.reset();
        planner_.reset();
        auto robot = test_helpers::defaultRobotParameters();
        parameters.insert(parameters.end(), robot.begin(), robot.end());
        rclcpp::NodeOptions options;
        options.parameter_overrides(parameters);
        node_ = std::make_shared<rclcpp::Node>("coordinator_test", options);
        planner_ = std::make_shared<CActionPlanner>(node_);
        coordinator_ = std::make_unique<CCoordinator>(node_, planner_);
    }
    void feedback(uint8_t type) {
        MovementRequest request;
        request.type = type;
        coordinator_->movementTypeActualReceived(request);
    }
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CActionPlanner> planner_;
    std::unique_ptr<CCoordinator> coordinator_;
};

TEST_F(CoordinatorTest, ReleasesExpiredLockAndMatchingMovement) {
    create();
    CoordinatorTestAccess::submit(*coordinator_, MovementRequest::SEQUENCE_WAITING, 10.0);
    feedback(MovementRequest::SEQUENCE_WAITING);
    coordinator_->update();
    EXPECT_TRUE(CoordinatorTestAccess::locked(*coordinator_));
    CoordinatorTestAccess::expire(*coordinator_);
    coordinator_->update();
    EXPECT_FALSE(CoordinatorTestAccess::locked(*coordinator_));
    EXPECT_EQ(CoordinatorTestAccess::actualType(*coordinator_), MovementRequest::NO_REQUEST);
}

TEST_F(CoordinatorTest, NewRequestReplacesExpiredDeadline) {
    create();
    CoordinatorTestAccess::submit(*coordinator_, MovementRequest::SEQUENCE_WAITING, 10.0);
    CoordinatorTestAccess::expire(*coordinator_);
    CoordinatorTestAccess::submit(*coordinator_, MovementRequest::SEQUENCE_LOOK, 20.0);
    feedback(MovementRequest::SEQUENCE_LOOK);
    coordinator_->update();
    EXPECT_TRUE(CoordinatorTestAccess::locked(*coordinator_));
    EXPECT_EQ(CoordinatorTestAccess::actualType(*coordinator_), MovementRequest::SEQUENCE_LOOK);
    CoordinatorTestAccess::expire(*coordinator_);
    coordinator_->update();
    EXPECT_EQ(CoordinatorTestAccess::actualType(*coordinator_), MovementRequest::NO_REQUEST);
}

TEST_F(CoordinatorTest, ExpirationPreservesNewerFeedback) {
    create();
    CoordinatorTestAccess::submit(*coordinator_, MovementRequest::SEQUENCE_WAITING, 10.0);
    feedback(MovementRequest::SEQUENCE_LOOK);
    CoordinatorTestAccess::expire(*coordinator_);
    coordinator_->update();
    EXPECT_FALSE(CoordinatorTestAccess::locked(*coordinator_));
    EXPECT_EQ(CoordinatorTestAccess::actualType(*coordinator_), MovementRequest::SEQUENCE_LOOK);
}

TEST_F(CoordinatorTest, WalkingAndRunningClearPreviousDeadline) {
    create();
    for (auto type : {MovementRequest::CONTINUOUS_MOVE, MovementRequest::CONTINUOUS_RUNNING}) {
        CoordinatorTestAccess::submit(*coordinator_, MovementRequest::SEQUENCE_WAITING, 10.0);
        CoordinatorTestAccess::expire(*coordinator_);
        CoordinatorTestAccess::submit(*coordinator_, type, 0.0);
        feedback(type);
        coordinator_->update();
        EXPECT_FALSE(CoordinatorTestAccess::locked(*coordinator_));
        EXPECT_EQ(CoordinatorTestAccess::actualType(*coordinator_), type);
    }
}

TEST_F(CoordinatorTest, NonpositiveDurationsExpireOnNextUpdate) {
    create();
    for (double seconds : {0.0, -1.0}) {
        CoordinatorTestAccess::submit(*coordinator_, MovementRequest::SEQUENCE_WAITING, seconds);
        feedback(MovementRequest::SEQUENCE_WAITING);
        EXPECT_TRUE(CoordinatorTestAccess::locked(*coordinator_));
        coordinator_->update();
        EXPECT_FALSE(CoordinatorTestAccess::locked(*coordinator_));
        EXPECT_EQ(CoordinatorTestAccess::actualType(*coordinator_), MovementRequest::NO_REQUEST);
    }
}

TEST_F(CoordinatorTest, RejectsNonfiniteDurationWithoutReplacingDeadline) {
    create();
    CoordinatorTestAccess::submit(*coordinator_, MovementRequest::SEQUENCE_WAITING, 10.0);
    feedback(MovementRequest::SEQUENCE_WAITING);
    for (double seconds : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity()}) {
        EXPECT_THROW(CoordinatorTestAccess::submit(*coordinator_, MovementRequest::SEQUENCE_LOOK, seconds),
                     std::invalid_argument);
    }
    CoordinatorTestAccess::expire(*coordinator_);
    coordinator_->update();
    EXPECT_EQ(CoordinatorTestAccess::actualType(*coordinator_), MovementRequest::NO_REQUEST);
}

TEST_F(CoordinatorTest, CanDestroyCoordinatorWithPendingDeadline) {
    create();
    CoordinatorTestAccess::submit(*coordinator_, MovementRequest::SEQUENCE_WAITING, 10.0);
    EXPECT_TRUE(CoordinatorTestAccess::locked(*coordinator_));
    EXPECT_NO_THROW(coordinator_.reset());
}

TEST_F(CoordinatorTest, RequiresEachCoordinatorParameter) {
    for (const char* name : {"max_velocity_linear", "max_velocity_rotation", "body_factor_height",
                             "joystick_deadzone", "min_body_height", "max_body_height"}) {
        SCOPED_TRACE(name);
        auto parameters = test_helpers::defaultCoordinatorParameters();
        std::erase_if(parameters, [name](const auto& parameter) { return parameter.get_name() == name; });
        try {
            create(parameters);
            FAIL() << "Missing parameter was accepted: " << name;
        } catch (const rclcpp::exceptions::UninitializedStaticallyTypedParameterException& error) {
            EXPECT_NE(std::string(error.what()).find(name), std::string::npos);
        }
    }
}

TEST_F(CoordinatorTest, RejectsInvalidParameterRanges) {
    const std::vector<rclcpp::Parameter> invalid = {
        rclcpp::Parameter("max_velocity_linear", 0.0),   rclcpp::Parameter("max_velocity_linear", -0.01),
        rclcpp::Parameter("max_velocity_rotation", 0.0), rclcpp::Parameter("max_velocity_rotation", -0.01),
        rclcpp::Parameter("body_factor_height", -0.01),  rclcpp::Parameter("joystick_deadzone", -0.01),
        rclcpp::Parameter("joystick_deadzone", 1.0),     rclcpp::Parameter("min_body_height", 0.01),
        rclcpp::Parameter("max_body_height", -0.01)};
    for (const auto& value : invalid) {
        SCOPED_TRACE(value.get_name());
        auto parameters = test_helpers::defaultCoordinatorParameters();
        for (auto& parameter : parameters) {
            if (parameter.get_name() == value.get_name()) parameter = value;
        }
        try {
            create(parameters);
            FAIL() << "Invalid parameter was accepted";
        } catch (const std::invalid_argument& error) {
            EXPECT_NE(std::string(error.what()).find(value.get_name()), std::string::npos);
        }
    }
}

TEST_F(CoordinatorTest, RequiresFiniteCoordinatorParameters) {
    for (const char* name : {"max_velocity_linear", "max_velocity_rotation", "body_factor_height",
                             "joystick_deadzone", "min_body_height", "max_body_height"}) {
        SCOPED_TRACE(name);
        for (double value :
             {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
              -std::numeric_limits<double>::infinity()}) {
            auto parameters = test_helpers::defaultCoordinatorParameters();
            for (auto& parameter : parameters) {
                if (parameter.get_name() == name) parameter = rclcpp::Parameter(name, value);
            }
            EXPECT_THROW(create(parameters), std::invalid_argument);
        }
    }
}

TEST_F(CoordinatorTest, AcceptsValidParameterBoundaries) {
    auto parameters = test_helpers::defaultCoordinatorParameters();
    for (auto& parameter : parameters) {
        const auto& name = parameter.get_name();
        if (name == "body_factor_height" || name == "joystick_deadzone" || name == "min_body_height" ||
            name == "max_body_height") {
            parameter = rclcpp::Parameter(name, 0.0);
        }
    }
    EXPECT_NO_THROW(create(parameters));
}

}  // namespace brain
