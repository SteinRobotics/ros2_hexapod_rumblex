#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "gait/gait_yaw_sequence.hpp"
#include "movement_request.hpp"
#include "rumblex_utils/body_pose.hpp"
#include "test_helpers.hpp"

namespace {

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using mp_units::angular::unit_symbols::deg;
using namespace brain;

class YawSequenceTest : public ::testing::TestWithParam<uint8_t> {
   protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        rclcpp::NodeOptions options;
        options.parameter_overrides(test_helpers::defaultRobotParameters());
        node = std::make_shared<rclcpp::Node>("yaw_sequence_test", options);
        model = std::make_shared<CPoseModel>(node);
        resetPose();
        auto params = test_helpers::makeDeclaredParameters(node);
        gait = std::make_unique<CYawSequenceGait>(node, model, params.watch.head_max_yaw, 0.0 * deg);
    }
    void TearDown() override {
        gait.reset();
        model.reset();
        node.reset();
        rclcpp::shutdown();
    }
    void resetPose() {
        model->moveTorso(model->getStandingToePositions(), CPose(0.002, -0.003, 0.001, 2.0, -1.0, 3.0));
        model->setHeadOrientation(COrientation(1.0, 4.0, -2.0));
    }
    uint8_t direction() const {
        return GetParam();
    }
    std::shared_ptr<rclcpp::Node> node;
    std::shared_ptr<CPoseModel> model;
    std::unique_ptr<ISequenceGait> gait;
};

TEST_P(YawSequenceTest, ExcursionPreservesPoseComponentsAndReturnsExactlyToOrigin) {
    const auto origin = bodyPose(*model);
    const double sign = direction() == MovementRequest::CLOCKWISE ? -1.0 : 1.0;
    EXPECT_FALSE(gait->update());
    gait->start(2.0 * s, direction());
    EXPECT_EQ(bodyPose(*model), origin);
    double peak = 0.0, minimum = 0.0;
    for (int tick = 1; tick <= 20; ++tick) {
        ASSERT_TRUE(gait->update());
        auto pose = bodyPose(*model);
        const double head_delta = pose.head_orientation.yaw - origin.head_orientation.yaw;
        peak = std::max(peak, sign * head_delta);
        minimum = std::min(minimum, sign * head_delta);
        EXPECT_EQ(pose.torso_pose, origin.torso_pose);
        if (tick == 1) {
            EXPECT_GT(sign * head_delta, 0.0);
        }
        EXPECT_LE(std::abs(head_delta), 30.0 + 1e-12);
        // All components except the commanded yaw axes stay at their origins.
        pose.head_orientation.yaw = origin.head_orientation.yaw;
        pose.torso_pose.orientation.yaw = origin.torso_pose.orientation.yaw;
        EXPECT_EQ(pose, origin);
    }
    EXPECT_GT(peak, 29.0);
    EXPECT_LT(minimum, -29.0);
    EXPECT_EQ(gait->state(), EGaitState::Stopped);
    EXPECT_EQ(bodyPose(*model), origin);
    EXPECT_FALSE(gait->update());
    EXPECT_EQ(bodyPose(*model), origin);
}

TEST_P(YawSequenceTest, DurationClampsAndCompletesOnFirstTickAtOrAfterEndpoint) {
    for (double duration : {-2.0, 0.0, 0.5, 1.0, 1.25, 2.0, 3.0}) {
        SCOPED_TRACE(duration);
        resetPose();
        const auto origin = bodyPose(*model);
        gait->start(duration * s, direction());
        const int ticks = static_cast<int>(std::ceil(std::max(duration, 1.0) / 0.1));
        for (int i = 1; i <= ticks; ++i) {
            ASSERT_TRUE(gait->update());
            EXPECT_EQ(gait->state(), i == ticks ? EGaitState::Stopped : EGaitState::Running);
        }
        EXPECT_EQ(bodyPose(*model), origin);
    }
}

TEST_P(YawSequenceTest, StopRequestsFinishExcursionAndRestartCapturesNewOrigin) {
    const auto origin = bodyPose(*model);
    gait->start(1.0 * s, direction());
    for (int i = 0; i < 3; ++i) ASSERT_TRUE(gait->update());
    const auto in_flight = bodyPose(*model);
    gait->requestStop();
    EXPECT_EQ(bodyPose(*model), in_flight);
    EXPECT_EQ(gait->state(), EGaitState::Running);
    gait->cancelStop();
    EXPECT_EQ(bodyPose(*model), in_flight);
    for (int i = 0; i < 7; ++i) ASSERT_TRUE(gait->update());
    EXPECT_EQ(gait->state(), EGaitState::Stopped);
    EXPECT_EQ(bodyPose(*model), origin);

    model->moveTorso(model->getToePositions(), CPose(0.001, 0.002, 0.003, -1.0, 2.0, -4.0));
    model->setHeadOrientation(COrientation(2.0, -3.0, 5.0));
    const auto new_origin = bodyPose(*model);
    const uint8_t opposite = direction() == MovementRequest::CLOCKWISE ? MovementRequest::ANTICLOCKWISE
                                                                       : MovementRequest::CLOCKWISE;
    gait->start(1.0 * s, opposite);
    EXPECT_EQ(bodyPose(*model), new_origin);
    ASSERT_TRUE(gait->update());
    const double sign = opposite == MovementRequest::CLOCKWISE ? -1.0 : 1.0;
    EXPECT_GT(sign * (bodyPose(*model).head_orientation.yaw - new_origin.head_orientation.yaw), 0.0);
    for (int i = 0; i < 9; ++i) ASSERT_TRUE(gait->update());
    EXPECT_EQ(gait->state(), EGaitState::Stopped);
    EXPECT_EQ(bodyPose(*model), new_origin);
}

INSTANTIATE_TEST_SUITE_P(WatchBothDirections, YawSequenceTest,
                         ::testing::Values(static_cast<uint8_t>(MovementRequest::CLOCKWISE),
                                           static_cast<uint8_t>(MovementRequest::ANTICLOCKWISE)));

}  // namespace
