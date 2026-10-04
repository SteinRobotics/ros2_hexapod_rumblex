#include <gtest/gtest.h>

#include "gait/gait_move_combined.hpp"
#include "gait/gait_running.hpp"
#include "movement_request.hpp"
#include "test_helpers.hpp"

using namespace brain;

class LocomotionTransitionTest : public ::testing::TestWithParam<MovementRequest::Type> {
   protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        rclcpp::NodeOptions options;
        options.parameter_overrides(test_helpers::defaultRobotParameters());
        node_ = std::make_shared<rclcpp::Node>("locomotion_transition_test", options);
        model_ = std::make_shared<CPoseModel>(node_);
        model_->moveTorso(model_->getStandingToePositions());
        params_ = test_helpers::makeDeclaredParameters(node_);
        gait_ = makeGait(model_);
        forward_.linear.x = 0.01;
    }
    void TearDown() override {
        gait_.reset();
        model_.reset();
        node_.reset();
        rclcpp::shutdown();
    }
    std::unique_ptr<IContinuousGait> makeGait(const std::shared_ptr<CPoseModel>& model) {
        if (GetParam() == MovementRequest::CONTINUOUS_RUNNING) {
            return std::make_unique<CRunningGait>(node_, model, params_.running);
        }
        return std::make_unique<CMoveCombinedGait>(node_, model, params_.wave, params_.ripple, params_.tripod,
                                                   params_.move_combined);
    }
    void startRunning() {
        gait_->start(0.0, MovementRequest::CLOCKWISE);
        for (int i = 0; i < 1000 && gait_->state() != EGaitState::Running; ++i) {
            gait_->update(forward_, CPose(), COrientation());
        }
        ASSERT_EQ(gait_->state(), EGaitState::Running);
    }
    void expectStanding(const CPose& torso = CPose()) {
        EXPECT_EQ(model_->getToePositions(), model_->getStandingToePositions());
        EXPECT_EQ(model_->getTorsoPose(), torso);
        EXPECT_EQ(model_->getHeadOrientation(), COrientation());
    }
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CPoseModel> model_;
    Parameters params_;
    std::unique_ptr<IContinuousGait> gait_;
    geometry_msgs::msg::Twist forward_;
};

TEST_P(LocomotionTransitionTest, StoppingBeforeFirstSegmentHoldsActualPose) {
    model_->setHeadOrientation(COrientation(5.0, 10.0, 15.0));
    const auto before = model_->getToePositions();
    const auto head = model_->getHeadOrientation();
    gait_->start(0.0, MovementRequest::CLOCKWISE);
    gait_->requestStop();
    ASSERT_EQ(gait_->state(), EGaitState::StopPending);
    const CPose torso(0.0, 0.0, 0.01, 0.0, 0.0, 0.0);
    EXPECT_FALSE(gait_->update(geometry_msgs::msg::Twist(), torso, COrientation()));
    EXPECT_EQ(gait_->state(), EGaitState::Stopped);
    EXPECT_EQ(model_->getToePositions(), before);
    EXPECT_EQ(model_->getHeadOrientation(), head);
    EXPECT_EQ(model_->getTorsoPose(), CPose());
    EXPECT_FALSE(gait_->update(forward_, torso, COrientation()));
}

TEST_P(LocomotionTransitionTest, RepeatedStopDoesNotOverwriteStartupResumeState) {
    gait_->start(0.0, MovementRequest::CLOCKWISE);
    gait_->requestStop();
    gait_->requestStop();
    gait_->cancelStop();
    EXPECT_EQ(gait_->state(), EGaitState::Starting);
    EXPECT_TRUE(gait_->update(forward_, CPose(), COrientation()));
}

TEST_P(LocomotionTransitionTest, RunningStopCanBeCancelledWithoutRestartingTrajectory) {
    startRunning();
    ASSERT_EQ(gait_->state(), EGaitState::Running);
    auto control_model = std::make_shared<CPoseModel>(*model_);
    control_model->moveTorso(control_model->getStandingToePositions());
    control_model->setHeadOrientation(COrientation());
    auto control = makeGait(control_model);
    control->start(0.0, MovementRequest::CLOCKWISE);
    for (int i = 0; i < 1000 && control->state() != EGaitState::Running; ++i) {
        control->update(forward_, CPose(), COrientation());
    }
    ASSERT_EQ(control->state(), EGaitState::Running);
    const auto before = model_->getToePositions();
    gait_->requestStop();
    gait_->requestStop();
    gait_->cancelStop();
    EXPECT_EQ(gait_->state(), EGaitState::Running);
    EXPECT_EQ(model_->getToePositions(), before);
    for (int i = 0; i < 8; ++i) {
        EXPECT_TRUE(gait_->update(forward_, CPose(), COrientation()));
        EXPECT_TRUE(control->update(forward_, CPose(), COrientation()));
        EXPECT_EQ(model_->getToePositions(), control_model->getToePositions());
        EXPECT_EQ(model_->getHeadOrientation(), control_model->getHeadOrientation());
    }
}

TEST_P(LocomotionTransitionTest, StopsUsingStoredVelocityAfterInputBecomesZero) {
    startRunning();
    ASSERT_EQ(gait_->state(), EGaitState::Running);
    gait_->requestStop();
    for (int i = 0; i < 1000 && gait_->state() != EGaitState::Stopped; ++i) {
        gait_->update(geometry_msgs::msg::Twist(), CPose(), COrientation());
    }
    EXPECT_EQ(gait_->state(), EGaitState::Stopped);
    expectStanding();
}

TEST_P(LocomotionTransitionTest, RestartAfterStopWaitsForNewVelocity) {
    startRunning();
    gait_->requestStop();
    for (int i = 0; i < 1000 && gait_->state() != EGaitState::Stopped; ++i) {
        gait_->update(forward_, CPose(), COrientation());
    }
    ASSERT_EQ(gait_->state(), EGaitState::Stopped);
    gait_->start(0.0, MovementRequest::CLOCKWISE);
    EXPECT_FALSE(gait_->update(geometry_msgs::msg::Twist(), CPose(), COrientation()));
    EXPECT_EQ(gait_->state(), EGaitState::Starting);
    gait_->requestStop();
    gait_->update(geometry_msgs::msg::Twist(), CPose(), COrientation());
    expectStanding();
}

INSTANTIATE_TEST_SUITE_P(WalkingAndRunning, LocomotionTransitionTest,
                         ::testing::Values(MovementRequest::CONTINUOUS_MOVE,
                                           MovementRequest::CONTINUOUS_RUNNING));
