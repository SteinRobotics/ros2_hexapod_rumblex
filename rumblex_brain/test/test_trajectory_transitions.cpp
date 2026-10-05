#include <gtest/gtest.h>

#include "gait/gait_controller.hpp"
#include "gait/stride_planner.hpp"
#include "rcl/time.h"
#include "test_helpers.hpp"

using namespace brain;

TEST(Trajectory, RestEndpointsMatchPositionVelocityAndAcceleration) {
    constexpr double h = 1e-5;
    for (auto curve : {trajectoryProgress, trajectoryLift}) {
        for (double endpoint : {0.0, 1.0}) {
            const double direction = endpoint == 0.0 ? 1.0 : -1.0;
            const double p = curve(endpoint);
            const double p1 = curve(endpoint + direction * h);
            const double p2 = curve(endpoint + direction * 2.0 * h);
            EXPECT_NEAR((p1 - p) / h, 0.0, 1e-7);
            EXPECT_NEAR((p2 - 2.0 * p1 + p) / (h * h), 0.0, 0.004);
        }
    }
    EXPECT_DOUBLE_EQ(trajectoryProgress(0.0), 0.0);
    EXPECT_DOUBLE_EQ(trajectoryProgress(1.0), 1.0);
    EXPECT_DOUBLE_EQ(trajectoryLift(0.5), 1.0);
}

class TrajectoryTransitionTest : public ::testing::Test {
   protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        rclcpp::NodeOptions options;
        options.parameter_overrides(test_helpers::defaultRobotParameters());
        node = std::make_shared<rclcpp::Node>("trajectory_transitions", options);
        model = std::make_shared<CPoseModel>(node);
        model->moveTorso(model->getStandingToePositions());
    }
    void TearDown() override {
        rclcpp::shutdown();
    }
    std::shared_ptr<rclcpp::Node> node;
    std::shared_ptr<CPoseModel> model;
};

std::vector<StridePattern> patterns() {
    using enum ELegIndex;
    return {{{{RightBack}, {RightMid}, {RightFront}, {LeftBack}, {LeftMid}, {LeftFront}},
             0.02 * units::m,
             0.025 * units::m,
             5.0 * units::deg,
             40.0},
            {{{RightBack, LeftFront}, {RightMid, LeftMid}, {RightFront, LeftBack}},
             0.025 * units::m,
             0.03 * units::m,
             8.0 * units::deg,
             40.0},
            {{{RightFront, LeftMid, RightBack}, {LeftFront, RightMid, LeftBack}},
             0.03 * units::m,
             0.035 * units::m,
             10.0 * units::deg,
             60.0}};
}

TEST_F(TrajectoryTransitionTest, EveryPatternChangeAndReversalCompletesCurrentSwing) {
    for (const auto& source : patterns()) {
        for (const auto& destination : patterns()) {
            for (int interrupt_tick : {1, 4, 8}) {
                model->moveTorso(model->getStandingToePositions());
                model->setHeadOrientation(COrientation());
                auto reference_model = std::make_shared<CPoseModel>(*model);
                CStridePlanner planner(model), reference(reference_model);
                planner.start();
                reference.start();
                geometry_msgs::msg::Twist velocity;
                velocity.linear.x = 0.01;
                for (int i = 0; i < interrupt_tick; ++i) {
                    planner.update(source, velocity, 0.7, CPose(), 0.02);
                    reference.update(source, velocity, 0.7, CPose(), 0.02);
                }
                geometry_msgs::msg::Twist reverse;
                reverse.linear.y = -0.02;
                while (!planner.atBoundary()) {
                    planner.update(destination, reverse, 0.7, CPose(), 0.02);
                    reference.update(source, velocity, 0.7, CPose(), 0.02);
                    EXPECT_EQ(model->getToePositions(), reference_model->getToePositions());
                    EXPECT_EQ(model->getHeadOrientation(), reference_model->getHeadOrientation());
                }
                const auto touchdown = model->getToePositions();
                planner.update(destination, reverse, 0.7, CPose(), 0.02);
                for (const auto& [leg, toe] : touchdown) {
                    const auto now = model->getToePositions().at(leg);
                    EXPECT_LT(std::abs((now.x - toe.x).numerical_value_in(units::m)), 0.001);
                    EXPECT_LT(std::abs((now.y - toe.y).numerical_value_in(units::m)), 0.001);
                    EXPECT_LT(std::abs((now.z - toe.z).numerical_value_in(units::m)), 0.002);
                }
            }
        }
    }
}

TEST_F(TrajectoryTransitionTest, StartupUsesDisplacedFeetTorsoAndHead) {
    auto toes = model->getToePositions();
    for (auto& [index, toe] : toes) {
        toe.x += 0.007 * units::m;
        toe.z += 0.009 * units::m;
    }
    const CPose torso(0.002, -0.003, 0.004, 1.0, 2.0, 3.0);
    const COrientation head(0.0, 5.0, 10.0);
    model->moveTorso(toes, torso);
    model->setHeadOrientation(head);
    CStridePlanner planner(model);
    planner.start();
    EXPECT_EQ(model->getToePositions(), toes);
    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = 0.01;
    planner.update(patterns().front(), velocity, 0.7, CPose(), 0.02);
    for (const auto& [index, toe] : toes) {
        const auto now = model->getToePositions().at(index);
        EXPECT_LT(std::abs((now.x - toe.x).numerical_value_in(units::m)), 0.001);
        EXPECT_LT(std::abs((now.z - toe.z).numerical_value_in(units::m)), 0.002);
    }
    EXPECT_GT(model->getHeadOrientation().yaw, 9.0 * units::deg);
    EXPECT_GT(model->getTorsoPose().position.z, 0.003 * units::m);
}

TEST_F(TrajectoryTransitionTest, StopAndResumeDuringSettlementNeverMutatePoseOnRequest) {
    CStridePlanner planner(model);
    planner.start();
    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = 0.01;
    for (int i = 0; i < 4; ++i) planner.update(patterns().front(), velocity, 0.7, CPose(), 0.02);
    planner.requestStop();
    for (int i = 0; i < 100 && planner.state() != EGaitState::Stopping; ++i)
        planner.update(patterns().front(), velocity, 0.7, CPose(), 0.02);
    ASSERT_EQ(planner.state(), EGaitState::Stopping);
    const auto before = model->getToePositions();
    planner.cancelStop();
    EXPECT_EQ(model->getToePositions(), before);
    EXPECT_EQ(planner.state(), EGaitState::Running);
    planner.requestStop();
    for (int i = 0; i < 1000 && planner.state() != EGaitState::Stopped; ++i)
        planner.update(patterns().front(), geometry_msgs::msg::Twist(), 0.7, CPose(), 0.02);
    EXPECT_EQ(planner.state(), EGaitState::Stopped);
    EXPECT_EQ(model->getToePositions(), model->getStandingToePositions());
}

TEST_F(TrajectoryTransitionTest, AllBehaviorPairsHandOffFromLastCommandedPose) {
    const std::vector<MovementRequest::Type> types = {
        MovementRequest::SEQUENCE_STAND_UP,  MovementRequest::SEQUENCE_LAYDOWN,
        MovementRequest::CONTINUOUS_MOVE,    MovementRequest::CONTINUOUS_RUNNING,
        MovementRequest::SEQUENCE_BODY_ROLL, MovementRequest::SEQUENCE_CLAP,
        MovementRequest::SEQUENCE_HIGH_FIVE, MovementRequest::SEQUENCE_LEGS_WAVE,
        MovementRequest::SEQUENCE_LOOK,      MovementRequest::SEQUENCE_WATCH,
        MovementRequest::SEQUENCE_WAITING,   MovementRequest::SEQUENCE_TESTLEGS,
        MovementRequest::SINGLE_POSE,        MovementRequest::CONTINUOUS_POSE};
    CGaitController controller(node, model);
    ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()), RCL_RET_OK);
    int64_t time = 1000000000;
    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = 0.01;
    auto tick = [&] {
        time += 100000000;
        EXPECT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(), time), RCL_RET_OK);
        controller.updateSelectedGait(velocity);
    };
    for (auto source : types) {
        for (auto destination : types) {
            SCOPED_TRACE(std::to_string(source) + " -> " + std::to_string(destination));
            MovementRequest request;
            request.type = source;
            request.duration_s = 2.0;
            controller.setGait(request);
            for (int i = 0; i < 1500 && controller.hasPendingGait(); ++i) tick();
            ASSERT_FALSE(controller.hasPendingGait());
            for (int i = 0; i < 4; ++i) tick();
            const auto before_request = model->getToePositions();
            request.type = destination;
            controller.setGait(request);
            EXPECT_EQ(model->getToePositions(), before_request);
            const bool starts_now = source != destination && !controller.hasPendingGait();
            bool handoff = false;
            for (int i = 0; i < 1500; ++i) {
                const auto before = model->getToePositions();
                const bool ready = controller.stopped();
                tick();
                if ((ready || starts_now) && !controller.hasPendingGait()) {
                    for (const auto& [leg, toe] : before) {
                        const auto now = model->getToePositions().at(leg);
                        EXPECT_LT(std::abs((now.x - toe.x).numerical_value_in(units::m)), 0.01);
                        EXPECT_LT(std::abs((now.y - toe.y).numerical_value_in(units::m)), 0.01);
                        EXPECT_LT(std::abs((now.z - toe.z).numerical_value_in(units::m)), 0.01);
                    }
                    handoff = true;
                    break;
                }
                if (!controller.hasPendingGait()) {
                    handoff = true;
                    break;
                }
            }
            EXPECT_TRUE(handoff);
            EXPECT_EQ(controller.currentGait(), destination);
        }
    }
}

TEST_F(TrajectoryTransitionTest, IdleSettlesAndResumesButExplicitStopDoesNotResume) {
    CStridePlanner planner(model);
    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = 0.01;
    planner.start();
    for (int i = 0; i < 4; ++i) planner.update(patterns().front(), velocity, 0.7, CPose(), 0.02);
    for (int i = 0; i < 1000 && planner.state() != EGaitState::Stopped; ++i)
        planner.update(patterns().front(), geometry_msgs::msg::Twist(), 0.7, CPose(), 0.02);
    ASSERT_EQ(planner.state(), EGaitState::Stopped);
    EXPECT_EQ(model->getToePositions(), model->getStandingToePositions());
    EXPECT_TRUE(planner.update(patterns().front(), velocity, 0.7, CPose(), 0.02));
    planner.requestStop();
    for (int i = 0; i < 1000 && planner.state() != EGaitState::Stopped; ++i)
        planner.update(patterns().front(), velocity, 0.7, CPose(), 0.02);
    ASSERT_EQ(planner.state(), EGaitState::Stopped);
    EXPECT_FALSE(planner.update(patterns().front(), velocity, 0.7, CPose(), 0.02));
}

TEST_F(TrajectoryTransitionTest, PatternChangeRetainsCyclePhaseInsteadOfRestartingFirstGroup) {
    CStridePlanner planner(model);
    planner.start();
    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = 0.01;
    for (int segment = 0; segment < 2; ++segment) {
        do {
            planner.update(patterns()[0], velocity, 0.7, CPose(), 0.02);
        } while (!planner.atBoundary());
    }
    ASSERT_TRUE(planner.atBoundary());
    planner.update(patterns()[1], velocity, 0.7, CPose(), 0.02);
    const auto standing = model->getStandingToePositions();
    for (const auto& [index, toe] : model->getToePositions()) {
        if (index == ELegIndex::RightMid || index == ELegIndex::LeftMid)
            EXPECT_GT(toe.z, standing.at(index).z);
        else
            EXPECT_EQ(toe.z, standing.at(index).z);
    }
}
