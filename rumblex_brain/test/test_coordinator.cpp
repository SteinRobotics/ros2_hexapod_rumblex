#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <thread>
#include <tuple>

#include "requester/coordinator.hpp"
#include "test_helpers.hpp"

namespace brain {

using namespace mp_units;
using namespace mp_units::si::unit_symbols;

// Inspect scheduling in tests without exposing planner state to callers.
class ActionPlannerTestAccess {
   public:
    static size_t normalCount(const CActionPlanner& planner) {
        return planner.requests_normal_prio_.size();
    }
    static size_t highCount(const CActionPlanner& planner) {
        return planner.requests_high_prio_.size();
    }
    static CMovement& movement(CActionPlanner& planner) {
        return *planner.handler_movement_;
    }
    static const auto& activeRequests(const CActionPlanner& planner) {
        return planner.active_requests_;
    }
    static const auto& latestNormalRequest(const CActionPlanner& planner) {
        return planner.requests_normal_prio_.back();
    }
    static const auto& latestHighRequest(const CActionPlanner& planner) {
        return planner.requests_high_prio_.back();
    }
};

class CoordinatorTestAccess {
   public:
    static void submit(CCoordinator& coordinator, uint32_t type, double seconds) {
        coordinator.submitRequestMove({.type = type, .duration = seconds * s});
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
        std::erase_if(robot, [](const auto& p) { return p.get_name().starts_with("max_velocity_"); });
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

TEST_F(CoordinatorTest, GaitFailureRemovesOnlyItsSequenceInvocation) {
    create();
    auto publisher = node_->create_publisher<rumblex_interfaces::msg::BodyPose>(
        "body_pose_actual", rclcpp::QoS(1).transient_local());
    rclcpp::NodeOptions options;
    options.parameter_overrides(test_helpers::defaultKinematicsParameters());
    auto seed = std::make_shared<rclcpp::Node>("failure_seed", options);
    CPoseModel model(seed);
    model.moveTorso(model.getLaydownToePositions(), CPose());
    model.setHeadOrientation(
        COrientation(0.0, -node_->get_parameter("gait.generic.head_max_pitch_deg").as_double(), 0.0));
    publisher->publish(bodyPose(model));
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_);
    for (int i = 0; i < 20; ++i) {
        executor.spin_some();
        planner_->update();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    const auto first = planner_->nextInvocationId();
    const auto other = planner_->nextInvocationId();
    auto hold = std::make_shared<RequestMovementType>();
    hold->movementRequest.type = MovementRequest::HOLD_DISTANCE;
    hold->movementRequest.distance_m = 0.6 * m;
    auto own_next = std::make_shared<RequestTalking>();
    own_next->text = "must not run";
    auto unrelated = std::make_shared<RequestTalking>();
    unrelated->text = "other sequence";
    std::vector<std::string> spoken;
    auto speech = node_->create_subscription<std_msgs::msg::String>(
        "request_talking", 10, [&](const std_msgs::msg::String& msg) { spoken.push_back(msg.data); });
    planner_->request({hold}, Prio::Normal, first);
    planner_->request({own_next}, Prio::Normal, first);
    planner_->request({unrelated}, Prio::Normal, other);
    // Gait activation fails from a non-standing pose and reports through the normal movement path.
    for (int i = 0; i < 20; ++i) {
        executor.spin_some();
        planner_->update();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    EXPECT_EQ(ActionPlannerTestAccess::normalCount(*planner_), 0u);
    ASSERT_FALSE(ActionPlannerTestAccess::activeRequests(*planner_).empty());
    EXPECT_EQ(ActionPlannerTestAccess::activeRequests(*planner_).front(), unrelated);
    EXPECT_EQ(*hold->movementRequest.distance_m, 0.6 * m);
    EXPECT_EQ(std::count(spoken.begin(), spoken.end(), "must not run"), 0);
}

TEST_F(CoordinatorTest, CancellationAllowsSchedulingBeforeMotionStops) {
    create();
    auto publisher = node_->create_publisher<rumblex_interfaces::msg::BodyPose>(
        "body_pose_actual", rclcpp::QoS(1).transient_local());
    rclcpp::NodeOptions seed_options;
    seed_options.parameter_overrides(test_helpers::defaultKinematicsParameters());
    auto seed_node = std::make_shared<rclcpp::Node>("scheduler_seed", seed_options);
    CPoseModel model(seed_node);
    model.moveTorso(model.getLaydownToePositions(), CPose());
    publisher->publish(bodyPose(model));
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_);
    auto& movement = ActionPlannerTestAccess::movement(*planner_);
    // Deliver initialization without advancing the startup trajectory.
    for (int i = 0; i < 20; ++i) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    auto stand = std::make_shared<RequestMovementType>();
    stand->movementRequest.type = MovementRequest::SEQUENCE_STAND_UP;
    stand->movementRequest.duration_s = 1.0 * s;
    movement.run(stand);
    ASSERT_FALSE(movement.requestFinished());
    ASSERT_FALSE(movement.motionStopped());

    auto system = std::make_shared<RequestSystem>();
    planner_->request({system}, Prio::Highest);
    EXPECT_TRUE(movement.requestFinished());
    EXPECT_TRUE(movement.stopRequested());
    EXPECT_FALSE(movement.motionStopped());
    planner_->update();
    const auto& active = ActionPlannerTestAccess::activeRequests(*planner_);
    ASSERT_EQ(active.size(), 1u);
    EXPECT_EQ(active.front(), system);
    EXPECT_FALSE(movement.motionStopped());
}

TEST_F(CoordinatorTest, JoystickPoseModeActivatesGaitAndGroupsOrientationTargets) {
    create();
    rumblex_interfaces::msg::JoystickRequest request;
    request.button_start = true;
    coordinator_->joystickRequestReceived(request);
    request.button_start = false;
    request.left_stick_horizontal = 0.5;
    request.right_stick_horizontal = 0.7;
    coordinator_->joystickRequestReceived(request);
    const auto& group = ActionPlannerTestAccess::latestNormalRequest(*planner_);
    ASSERT_EQ(group.size(), 1u);
    const auto gait = std::dynamic_pointer_cast<RequestMovementType>(group[0]);
    ASSERT_TRUE(gait);
    EXPECT_EQ(gait->movementRequest.type, MovementRequest::CONTINUOUS_POSE);
    EXPECT_NEAR(gait->movementRequest.body_pose.torso_pose.position.y, -0.025, 1e-8);
    EXPECT_NEAR(gait->movementRequest.body_pose.head_orientation.yaw, -14.0, 1e-6);
    EXPECT_FALSE(CoordinatorTestAccess::locked(*coordinator_));
    request.left_stick_horizontal = 0.0;
    request.right_stick_horizontal = 0.0;
    coordinator_->joystickRequestReceived(request);
    // Releasing the sticks preserves the existing pose targets.
    EXPECT_EQ(ActionPlannerTestAccess::latestNormalRequest(*planner_).size(), 1u);
}

TEST_F(CoordinatorTest, JoystickDirectionsUseForwardLeftAndAnticlockwiseAxes) {
    create();
    for (float deflection : {-1.0f, 1.0f}) {
        rumblex_interfaces::msg::JoystickRequest request;
        request.left_stick_vertical = 0.5;
        request.left_stick_horizontal = deflection;
        request.right_stick_horizontal = deflection;
        coordinator_->joystickRequestReceived(request);
        const auto& group = ActionPlannerTestAccess::latestNormalRequest(*planner_);
        const auto velocity = std::dynamic_pointer_cast<RequestVelocity>(group.back());
        ASSERT_TRUE(velocity);
        EXPECT_GT(velocity->velocity.linear.x, 0.0);
        EXPECT_LT(deflection * velocity->velocity.linear.y, 0.0);
        EXPECT_LT(deflection * velocity->velocity.angular.z, 0.0);
    }
}

TEST_F(CoordinatorTest, JoystickDeadzoneBoundaryStopsWalkingWithoutPoseRequests) {
    auto parameters = test_helpers::defaultCoordinatorParameters();
    for (auto& parameter : parameters) {
        if (parameter.get_name() == "joystick_deadzone")
            parameter = rclcpp::Parameter("joystick_deadzone", 0.25);
    }
    create(parameters);
    feedback(MovementRequest::CONTINUOUS_MOVE);
    rumblex_interfaces::msg::JoystickRequest request;
    request.left_stick_vertical = 0.25;
    request.left_stick_horizontal = -0.25;
    request.right_stick_horizontal = 0.25;
    request.right_stick_vertical = -0.25;
    coordinator_->joystickRequestReceived(request);
    const auto& stopped = ActionPlannerTestAccess::latestNormalRequest(*planner_);
    ASSERT_EQ(stopped.size(), 1u);
    const auto velocity = std::dynamic_pointer_cast<RequestVelocity>(stopped.front());
    ASSERT_TRUE(velocity);
    EXPECT_EQ(velocity->velocity, geometry_msgs::msg::Twist());

    request.right_stick_vertical = 0.5;
    coordinator_->joystickRequestReceived(request);
    const auto& active = ActionPlannerTestAccess::latestNormalRequest(*planner_);
    ASSERT_EQ(active.size(), 2u);
    const auto gait = std::dynamic_pointer_cast<RequestMovementType>(active.front());
    ASSERT_TRUE(gait);
    EXPECT_EQ(gait->movementRequest.type, MovementRequest::CONTINUOUS_MOVE);
    EXPECT_GT(gait->movementRequest.body_pose.torso_pose.position.z, 0.0);
}

TEST_F(CoordinatorTest, CmdVelPreservesRosAxisSigns) {
    create();
    for (double sign : {-1.0, 1.0}) {
        geometry_msgs::msg::Twist command;
        command.linear.x = 0.01;
        command.linear.y = sign * 0.02;
        command.angular.z = sign * 0.03;
        coordinator_->cmdVelReceived(command);
        const auto& group = ActionPlannerTestAccess::latestHighRequest(*planner_);
        const auto velocity = std::dynamic_pointer_cast<RequestVelocity>(group.back());
        ASSERT_TRUE(velocity);
        EXPECT_EQ(velocity->velocity, command);
    }
}

TEST_F(CoordinatorTest, VoiceRunningUsesRequestedDirectionAndStops) {
    create();
    for (const auto& [phrase, x, y] :
         std::vector<std::tuple<std::string, double, double>>{{"renne nach vorne", 0.005, 0.0},
                                                              {"renn nach hinten", -0.005, 0.0},
                                                              {"flitze nach links", 0.0, 0.005},
                                                              {"flitz nach rechts", 0.0, -0.005}}) {
        coordinator_->speechRecognized(phrase);
        const auto& group = ActionPlannerTestAccess::latestNormalRequest(*planner_);
        ASSERT_GE(group.size(), 3u);
        const auto gait = std::dynamic_pointer_cast<RequestMovementType>(group[1]);
        const auto velocity = std::dynamic_pointer_cast<RequestVelocity>(group.back());
        ASSERT_TRUE(gait);
        ASSERT_TRUE(velocity);
        EXPECT_EQ(gait->movementRequest.type, MovementRequest::CONTINUOUS_RUNNING);
        EXPECT_DOUBLE_EQ(velocity->velocity.linear.x, x);
        EXPECT_DOUBLE_EQ(velocity->velocity.linear.y, y);
    }
    coordinator_->speechRecognized("stoppe bewegung");
    const auto& group = ActionPlannerTestAccess::latestNormalRequest(*planner_);
    const auto velocity = std::dynamic_pointer_cast<RequestVelocity>(group.back());
    ASSERT_TRUE(velocity);
    EXPECT_EQ(velocity->velocity, geometry_msgs::msg::Twist());
}

TEST_F(CoordinatorTest, NamedSequenceUsesNormalPriorityAndExactName) {
    create();
    const auto highCount = ActionPlannerTestAccess::highCount(*planner_);
    coordinator_->sequenceRequested("clap");
    const auto& group = ActionPlannerTestAccess::latestNormalRequest(*planner_);
    ASSERT_FALSE(group.empty());
    const auto gait = std::dynamic_pointer_cast<RequestMovementType>(group.front());
    ASSERT_TRUE(gait);
    EXPECT_EQ(gait->movementRequest.type, MovementRequest::SINGLE_POSE);
    EXPECT_EQ(ActionPlannerTestAccess::highCount(*planner_), highCount);
}

TEST_F(CoordinatorTest, NamedStandupUsesHighPriority) {
    create();
    const auto normalCount = ActionPlannerTestAccess::normalCount(*planner_);
    const auto highCount = ActionPlannerTestAccess::highCount(*planner_);
    coordinator_->sequenceRequested("standup");
    EXPECT_GT(ActionPlannerTestAccess::highCount(*planner_), highCount);
    EXPECT_EQ(ActionPlannerTestAccess::normalCount(*planner_), normalCount);
    const auto& group = ActionPlannerTestAccess::latestHighRequest(*planner_);
    const auto gait = std::find_if(group.begin(), group.end(), [](const auto& request) {
        return std::dynamic_pointer_cast<RequestMovementType>(request) != nullptr;
    });
    ASSERT_NE(gait, group.end());
    EXPECT_EQ(std::dynamic_pointer_cast<RequestMovementType>(*gait)->movementRequest.type,
              MovementRequest::SEQUENCE_STAND_UP);
}

TEST_F(CoordinatorTest, UnknownNamedSequenceDoesNotQueueActions) {
    create();
    const auto normalCount = ActionPlannerTestAccess::normalCount(*planner_);
    const auto highCount = ActionPlannerTestAccess::highCount(*planner_);
    coordinator_->sequenceRequested("missing_sequence");
    coordinator_->sequenceRequested("");
    EXPECT_EQ(ActionPlannerTestAccess::normalCount(*planner_), normalCount);
    EXPECT_EQ(ActionPlannerTestAccess::highCount(*planner_), highCount);
}

TEST_F(CoordinatorTest, VoiceClapSelectsPoseSequence) {
    create();
    coordinator_->speechRecognized("klatsche");
    const auto& group = ActionPlannerTestAccess::latestHighRequest(*planner_);
    const auto gait = std::dynamic_pointer_cast<RequestMovementType>(group.front());
    ASSERT_TRUE(gait);
    EXPECT_EQ(gait->movementRequest.type, MovementRequest::SINGLE_POSE);
}

TEST_F(CoordinatorTest, VoiceStandardDanceQueuesChoreography) {
    create();
    coordinator_->speechRecognized("tanze standardtanz");
    const auto& group = ActionPlannerTestAccess::latestHighRequest(*planner_);
    ASSERT_EQ(group.size(), 1u);
    EXPECT_TRUE(std::dynamic_pointer_cast<RequestMusic>(group.front()));
}

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
    CoordinatorTestAccess::submit(*coordinator_, MovementRequest::SEQUENCE_WATCH, 20.0);
    feedback(MovementRequest::SEQUENCE_WATCH);
    coordinator_->update();
    EXPECT_TRUE(CoordinatorTestAccess::locked(*coordinator_));
    EXPECT_EQ(CoordinatorTestAccess::actualType(*coordinator_), MovementRequest::SEQUENCE_WATCH);
    CoordinatorTestAccess::expire(*coordinator_);
    coordinator_->update();
    EXPECT_EQ(CoordinatorTestAccess::actualType(*coordinator_), MovementRequest::NO_REQUEST);
}

TEST_F(CoordinatorTest, ExpirationPreservesNewerFeedback) {
    create();
    CoordinatorTestAccess::submit(*coordinator_, MovementRequest::SEQUENCE_WAITING, 10.0);
    feedback(MovementRequest::SEQUENCE_WATCH);
    CoordinatorTestAccess::expire(*coordinator_);
    coordinator_->update();
    EXPECT_FALSE(CoordinatorTestAccess::locked(*coordinator_));
    EXPECT_EQ(CoordinatorTestAccess::actualType(*coordinator_), MovementRequest::SEQUENCE_WATCH);
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
        EXPECT_THROW(CoordinatorTestAccess::submit(*coordinator_, MovementRequest::SEQUENCE_WATCH, seconds),
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
