#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "gait/gait_move_combined.hpp"
#include "gait/gait_move_crawl.hpp"
#include "gait/gait_move_ripple.hpp"
#include "gait/gait_move_tripod.hpp"
#include "gait/gait_move_wave.hpp"
#include "movement_request.hpp"
#include "test_helpers.hpp"

namespace {

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
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
        forward_.linear.x = 0.01 * m / s;
    }
    void TearDown() override {
        gait_.reset();
        model_.reset();
        node_.reset();
        rclcpp::shutdown();
    }
    std::unique_ptr<IContinuousGait> makeGait(const std::shared_ptr<CPoseModel>& model) {
        if (GetParam() == MovementRequest::MOVE_TRIPOD) {
            return std::make_unique<CMoveTripodGait>(node_, model, params_.tripod);
        }
        if (GetParam() == MovementRequest::MOVE_CRAWL)
            return std::make_unique<CMoveCrawlGait>(node_, model, params_.crawl);
        if (GetParam() == MovementRequest::MOVE_RIPPLE)
            return std::make_unique<CMoveRippleGait>(node_, model, params_.ripple);
        if (GetParam() == MovementRequest::MOVE_WAVE)
            return std::make_unique<CMoveWaveGait>(node_, model, params_.wave);
        return std::make_unique<CMoveCombinedGait>(node_, model, params_.wave, params_.ripple, params_.tripod,
                                                   params_.move_combined);
    }
    void startRunning() {
        gait_->start(0.0 * s, MovementRequest::CLOCKWISE);
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
    Velocity forward_;
};

TEST_P(LocomotionTransitionTest, StoppingBeforeFirstSegmentHoldsActualPose) {
    model_->setHeadOrientation(COrientation(5.0, 10.0, 15.0));
    const auto before = model_->getToePositions();
    const auto head = model_->getHeadOrientation();
    gait_->start(0.0 * s, MovementRequest::CLOCKWISE);
    gait_->requestStop();
    ASSERT_EQ(gait_->state(), EGaitState::StopPending);
    const CPose torso(0.0, 0.0, 0.01, 0.0, 0.0, 0.0);
    EXPECT_FALSE(gait_->update(Velocity(), torso, COrientation()));
    EXPECT_EQ(gait_->state(), EGaitState::Stopped);
    EXPECT_EQ(model_->getToePositions(), before);
    EXPECT_EQ(model_->getHeadOrientation(), head);
    EXPECT_EQ(model_->getTorsoPose(), CPose());
    EXPECT_FALSE(gait_->update(forward_, torso, COrientation()));
}

TEST_P(LocomotionTransitionTest, RepeatedStopDoesNotOverwriteStartupResumeState) {
    gait_->start(0.0 * s, MovementRequest::CLOCKWISE);
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
    control->start(0.0 * s, MovementRequest::CLOCKWISE);
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
        gait_->update(Velocity(), CPose(), COrientation());
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
    gait_->start(0.0 * s, MovementRequest::CLOCKWISE);
    EXPECT_FALSE(gait_->update(Velocity(), CPose(), COrientation()));
    EXPECT_EQ(gait_->state(), EGaitState::Starting);
    gait_->requestStop();
    gait_->update(Velocity(), CPose(), COrientation());
    expectStanding();
}

TEST_P(LocomotionTransitionTest, SteadyVelocitySwingsExpectedLegGroups) {
    using enum ELegIndex;
    std::vector<std::vector<ELegIndex>> groups;
    switch (GetParam()) {
        case MovementRequest::MOVE_TRIPOD:
            groups = {{RightFront, LeftMid, RightBack}, {LeftFront, RightMid, LeftBack}};
            break;
        // The fixture requests 0.01 m/s with a 0.02 m/s limit: combined
        // must select ripple at this steady 50% demand for the whole cycle.
        case MovementRequest::MOVE_COMBINED:
        case MovementRequest::MOVE_RIPPLE:
            groups = {{RightBack, LeftFront}, {RightMid, LeftMid}, {RightFront, LeftBack}};
            break;
        case MovementRequest::MOVE_CRAWL:
            groups = {{RightBack, LeftBack}, {RightMid, LeftMid}, {RightFront, LeftFront}};
            break;
        case MovementRequest::MOVE_WAVE:
            groups = {{RightBack}, {RightMid}, {RightFront}, {LeftBack}, {LeftMid}, {LeftFront}};
            break;
        default:
            FAIL() << "Missing expected leg groups for gait " << GetParam();
    }
    gait_->start(0.0 * s, MovementRequest::CLOCKWISE);
    // At this low velocity all patterns use one-second segments. Check two
    // cycles to cover wraparound and steady support motion after startup.
    const auto standing = model_->getStandingToePositions();
    for (int cycle = 0; cycle < 2; ++cycle) {
        for (const auto& group : groups) {
            for (int tick = 1; tick <= 10; ++tick) {
                const auto before = model_->getToePositions();
                ASSERT_TRUE(gait_->updateTimed(forward_, CPose(), COrientation(), 0.1 * s));
                int airborne = 0;
                for (const auto& [index, toe] : model_->getToePositions()) {
                    const bool swing = std::ranges::contains(group, index);
                    EXPECT_EQ(toe.z > standing.at(index).z, swing && tick < 10);
                    if (toe.z > standing.at(index).z) ++airborne;
                    if (!swing) {
                        EXPECT_EQ(toe.z, standing.at(index).z);
                        EXPECT_EQ(toe.y, before.at(index).y);
                        if (cycle > 0) {
                            EXPECT_LT(toe.x, before.at(index).x);
                        }
                    }
                }
                EXPECT_EQ(airborne, tick < 10 ? static_cast<int>(group.size()) : 0);
            }
        }
    }
}

INSTANTIATE_TEST_SUITE_P(AllLocomotionGaits, LocomotionTransitionTest,
                         ::testing::Values(MovementRequest::MOVE_COMBINED, MovementRequest::MOVE_TRIPOD,
                                           MovementRequest::MOVE_RIPPLE, MovementRequest::MOVE_WAVE,
                                           MovementRequest::MOVE_CRAWL));

}  // namespace
