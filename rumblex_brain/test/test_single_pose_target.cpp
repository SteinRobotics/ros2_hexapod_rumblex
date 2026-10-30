#include <gtest/gtest.h>

#include "gait/gait_controller.hpp"
#include "parser/sequence_parser.hpp"
#include "rumblex_utils/body_pose.hpp"
#include "test_helpers.hpp"

namespace {
using namespace brain;
using namespace mp_units;
using namespace mp_units::si::unit_symbols;

class SinglePoseTargetTest : public ::testing::Test {
   protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        rclcpp::NodeOptions options;
        options.parameter_overrides(test_helpers::defaultRobotParameters());
        node = std::make_shared<rclcpp::Node>("single_pose_target", options);
        model = std::make_shared<CPoseModel>(node);
        model->moveTorso(model->getStandingToePositions(), CPose(0.003, -0.002, 0.001, 2, -3, 4));
        model->setHeadOrientation(COrientation(0, 6, 8));
        controller = std::make_unique<CGaitController>(node, model);
    }
    void TearDown() override {
        controller.reset();
        model.reset();
        node.reset();
        rclcpp::shutdown();
    }
    MovementRequest poseRequest() {
        MovementRequest request;
        request.type = MovementRequest::SINGLE_POSE;
        request.name = "SINGLE_POSE";
        request.duration_s = 1.0 * s;
        return request;
    }
    void advance(int samples) {
        for (int i = 0; i < samples; ++i)
            controller->updateSelectedGait(Velocity(), CPose(), COrientation(), 0.02 * s);
    }
    rclcpp::Node::SharedPtr node;
    std::shared_ptr<CPoseModel> model;
    std::unique_ptr<CGaitController> controller;
};

TEST_F(SinglePoseTargetTest, LookCatalogTurnsBothWaysAndRestoresStartingPose) {
    CSequenceParser parser(node);
    ASSERT_TRUE(parser.parseDirectory(RUMBLEX_SEQUENCES_DIRECTORY));
    for (bool left : {true, false}) {
        const auto name = left ? "look_left" : "look_right";
        SCOPED_TRACE(name);
        const auto sequence = parser.getSequence(name);
        ASSERT_TRUE(sequence);
        ASSERT_EQ(sequence->get().actionGroups.size(), 2u);
        ASSERT_EQ(sequence->get().actionGroups[0].size(), 2u);
        ASSERT_TRUE(parser.getSequenceForVoiceRequest(left ? "commandLookLeft" : "commandLookRight"));
        rumblex_interfaces::msg::JoystickRequest joystick;
        joystick.button_l1 = left;
        joystick.button_r1 = !left;
        ASSERT_EQ(parser.getSequenceForJoystickRequest(joystick)->get().name, name);
        for (int invocation = 0; invocation < 2; ++invocation) {
            model->setHeadOrientation(COrientation(3, 6 + invocation, 8 + invocation));
            const auto origin = bodyPose(*model);
            const double sign = left ? 1 : -1;
            for (size_t phase = 0; phase < 2; ++phase) {
                std::shared_ptr<RequestMovementType> movement;
                for (const auto& action : sequence->get().actionGroups[phase])
                    if (auto request = std::dynamic_pointer_cast<RequestMovementType>(action))
                        movement = request;
                ASSERT_TRUE(movement);
                EXPECT_EQ(movement->movementRequest.type, MovementRequest::SINGLE_POSE);
                EXPECT_EQ(movement->movementRequest.duration_s, 1.5 * s);
                const auto before = bodyPose(*model);
                ASSERT_TRUE(controller->setGait(movement->movementRequest));
                EXPECT_EQ(bodyPose(*model), before);
                double previous = before.head_orientation.yaw;
                // Alternating intervals exercise elapsed-time sampling (1.5 seconds total).
                for (int tick = 0; tick < 100; ++tick) {
                    controller->updateSelectedGait(Velocity(), CPose(), COrientation(),
                                                   (tick % 2 ? 0.02 : 0.01) * s);
                    auto actual = bodyPose(*model);
                    const double delta = actual.head_orientation.yaw - origin.head_orientation.yaw;
                    EXPECT_NEAR(actual.torso_pose.orientation.yaw - origin.torso_pose.orientation.yaw,
                                delta * 20.0 / 25.0, 1e-9);
                    EXPECT_GE(sign * delta, -1e-9);
                    EXPECT_LE(sign * delta, 25.0 + 1e-9);
                    EXPECT_GE((phase ? -sign : sign) * (actual.head_orientation.yaw - previous), -1e-9);
                    previous = actual.head_orientation.yaw;
                    actual.head_orientation.yaw = origin.head_orientation.yaw;
                    actual.torso_pose.orientation.yaw = origin.torso_pose.orientation.yaw;
                    EXPECT_EQ(actual, origin);
                    EXPECT_EQ(controller->motionStopped(), tick == 99);
                }
                if (phase == 0) {
                    EXPECT_NEAR(bodyPose(*model).head_orientation.yaw,
                                origin.head_orientation.yaw + sign * 25, 1e-9);
                    EXPECT_NEAR(bodyPose(*model).torso_pose.orientation.yaw,
                                origin.torso_pose.orientation.yaw + sign * 20, 1e-9);
                } else {
                    EXPECT_EQ(bodyPose(*model), origin);
                }
            }
        }
    }
}

TEST_F(SinglePoseTargetTest, PendingRotationDeltaCapturesEndpointAndAddsAllAxes) {
    auto first = poseRequest();
    first.body_pose.has_head_orientation = true;
    first.body_pose.head_orientation.yaw = 12;
    ASSERT_TRUE(controller->setGait(first));
    advance(20);
    auto rotate = poseRequest();
    rotate.save_pose = true;
    auto& delta = rotate.pose_delta.emplace();
    delta.head_rotation.roll = 2;
    delta.head_rotation.pitch = -3;
    delta.head_rotation.yaw = 25;
    delta.torso_rotation.roll = -1;
    delta.torso_rotation.pitch = 2;
    delta.torso_rotation.yaw = 20;
    ASSERT_TRUE(controller->setGait(rotate));
    advance(30);
    const auto origin = bodyPose(*model);
    advance(50);
    auto expected = origin;
    expected.head_orientation.roll += 2;
    expected.head_orientation.pitch -= 3;
    expected.head_orientation.yaw += 25;
    expected.torso_pose.orientation.roll -= 1;
    expected.torso_pose.orientation.pitch += 2;
    expected.torso_pose.orientation.yaw += 20;
    const auto actual = bodyPose(*model);
    EXPECT_EQ(actual.toe_positions, origin.toe_positions);
    EXPECT_EQ(actual.torso_pose.position, origin.torso_pose.position);
    EXPECT_NEAR(actual.head_orientation.roll, expected.head_orientation.roll, 1e-9);
    EXPECT_NEAR(actual.head_orientation.pitch, expected.head_orientation.pitch, 1e-9);
    EXPECT_NEAR(actual.head_orientation.yaw, expected.head_orientation.yaw, 1e-9);
    EXPECT_NEAR(actual.torso_pose.orientation.roll, expected.torso_pose.orientation.roll, 1e-9);
    EXPECT_NEAR(actual.torso_pose.orientation.pitch, expected.torso_pose.orientation.pitch, 1e-9);
    EXPECT_NEAR(actual.torso_pose.orientation.yaw, expected.torso_pose.orientation.yaw, 1e-9);
    auto restore = poseRequest();
    restore.restore_pose = true;
    ASSERT_TRUE(controller->setGait(restore));
    advance(50);
    EXPECT_EQ(bodyPose(*model), origin);
}

TEST_F(SinglePoseTargetTest, HighFiveCatalogRaisesHoldsAndRestoresEachInvocation) {
    CSequenceParser parser(node);
    ASSERT_TRUE(parser.parseFile(std::string(RUMBLEX_SEQUENCES_DIRECTORY) + "/high_five.yaml"));
    ASSERT_TRUE(parser.getSequenceForVoiceRequest("commandHighFive"));
    rumblex_interfaces::msg::JoystickRequest joystick;
    joystick.button_b = true;
    ASSERT_TRUE(parser.getSequenceForJoystickRequest(joystick));
    const auto sequence = parser.getSequence("high_five");
    ASSERT_TRUE(sequence);
    ASSERT_EQ(sequence->get().actionGroups.size(), 3u);
    ASSERT_EQ(sequence->get().actionGroups[0].size(), 2u);
    for (int invocation = 0; invocation < 2; ++invocation) {
        model->setHeadOrientation(COrientation(3, 6 + invocation, 8 + invocation));
        const auto origin = bodyPose(*model);
        auto raised = *model;
        raised.setLegAngles(ELegIndex::RightFront, CLegAngles(-20, 50, 60));
        raised.setHeadOrientation(COrientation(0, -20, 0));
        const auto raised_pose = bodyPose(raised);
        for (size_t i = 0; i < 3; ++i) {
            std::shared_ptr<RequestMovementType> movement;
            for (const auto& action : sequence->get().actionGroups[i])
                if (auto request = std::dynamic_pointer_cast<RequestMovementType>(action)) movement = request;
            ASSERT_TRUE(movement);
            ASSERT_EQ(movement->movementRequest.type, MovementRequest::SINGLE_POSE);
            const auto before = bodyPose(*model);
            ASSERT_TRUE(controller->setGait(movement->movementRequest));
            EXPECT_EQ(bodyPose(*model), before);
            advance(i == 1 ? 99 : 49);
            EXPECT_FALSE(controller->motionStopped());
            advance(1);
            EXPECT_TRUE(controller->motionStopped());
            const auto actual = bodyPose(*model);
            const auto expected = i == 2 ? origin : raised_pose;
            EXPECT_EQ(actual.head_orientation, expected.head_orientation);
            EXPECT_EQ(actual.torso_pose, origin.torso_pose);
            for (size_t leg = 0; leg < 6; ++leg) {
                EXPECT_NEAR(actual.toe_positions[leg].x, expected.toe_positions[leg].x, 1e-9);
                EXPECT_NEAR(actual.toe_positions[leg].y, expected.toe_positions[leg].y, 1e-9);
                EXPECT_NEAR(actual.toe_positions[leg].z, expected.toe_positions[leg].z, 1e-9);
            }
        }
    }
}

TEST_F(SinglePoseTargetTest, SavedPoseIsCapturedAfterPendingMotionAndConsumedOnReturn) {
    auto restore = poseRequest();
    restore.restore_pose = true;
    EXPECT_FALSE(controller->setGait(restore));
    auto first = poseRequest();
    first.body_pose.has_head_orientation = true;
    first.body_pose.head_orientation.yaw = 12;
    ASSERT_TRUE(controller->setGait(first));
    advance(20);
    auto raise = poseRequest();
    raise.save_pose = true;
    raise.joint_targets[0] = std::array<double, 3>{-20, 50, 60};
    ASSERT_TRUE(controller->setGait(raise));
    advance(30);
    const auto saved = bodyPose(*model);
    advance(50);
    ASSERT_TRUE(controller->setGait(restore));
    advance(50);
    EXPECT_EQ(bodyPose(*model), saved);
    EXPECT_FALSE(controller->setGait(restore));
}

TEST_F(SinglePoseTargetTest, ClapCatalogPreservesPhasesTimingAndStartingPose) {
    CSequenceParser parser(node);
    ASSERT_TRUE(parser.parseFile(std::string(RUMBLEX_SEQUENCES_DIRECTORY) + "/clap.yaml"));
    const auto sequence = parser.getSequence("clap");
    ASSERT_TRUE(sequence);
    ASSERT_EQ(sequence->get().actionGroups.size(), 20u);
    const auto origin = bodyPose(*model);
    std::map<ELegIndex, CLegAngles> raised_angles;
    double elapsed = 0;
    for (size_t i = 0; i < 20; ++i) {
        const auto request =
            std::dynamic_pointer_cast<RequestMovementType>(sequence->get().actionGroups[i].at(0));
        ASSERT_TRUE(request);
        ASSERT_EQ(request->movementRequest.type, MovementRequest::SINGLE_POSE);
        const auto before = bodyPose(*model);
        ASSERT_TRUE(controller->setGait(request->movementRequest));
        EXPECT_EQ(bodyPose(*model), before);
        const int samples = (i >= 6 && i < 18) ? 25 : 50;
        advance(samples - 1);
        EXPECT_FALSE(controller->motionStopped());
        advance(1);
        EXPECT_TRUE(controller->motionStopped());
        elapsed += samples * 0.02;
        const auto pose = bodyPose(*model);
        EXPECT_EQ(pose.head_orientation, origin.head_orientation);
        EXPECT_EQ(pose.torso_pose.orientation, origin.torso_pose.orientation);
        EXPECT_NEAR(pose.torso_pose.position.x, origin.torso_pose.position.x - (i == 19 ? 0.0 : 0.05), 1e-9);
        for (size_t leg = 0; leg < 6; ++leg) {
            if (i >= 6 && i < 18 && (leg == 0 || leg == 3)) continue;
            const double lift = (i == 1 && leg == 2) || (i == 3 && leg == 5)   ? 0.05
                                : (i >= 5 && i < 18 && (leg == 0 || leg == 3)) ? 0.1
                                                                               : 0.0;
            EXPECT_NEAR(pose.toe_positions[leg].x, origin.toe_positions[leg].x, 1e-8);
            EXPECT_NEAR(pose.toe_positions[leg].y, origin.toe_positions[leg].y, 1e-8);
            EXPECT_NEAR(pose.toe_positions[leg].z, origin.toe_positions[leg].z + lift, 1e-8);
        }
        if (i == 5) raised_angles = model->getLegAngles();
        if (i >= 6 && i < 18) {
            using mp_units::angular::unit_symbols::deg;
            const double offset = i % 2 == 0 ? 30 : 0;
            for (auto leg : {ELegIndex::RightFront, ELegIndex::LeftFront}) {
                EXPECT_NEAR((model->getLegAngles(leg).torso_coxa - raised_angles.at(leg).torso_coxa)
                                .numerical_value_in(deg),
                            leg == ELegIndex::RightFront ? -offset : offset, 1e-8);
            }
        }
    }
    EXPECT_DOUBLE_EQ(elapsed, 14.0);
}

TEST_F(SinglePoseTargetTest, PendingDeltaStartsAtOutgoingEndpointAndDoesNotLeak) {
    auto first = poseRequest();
    first.pose_delta.emplace().torso_translation.x = -0.02;
    const auto origin = bodyPose(*model);
    ASSERT_TRUE(controller->setGait(first));
    advance(20);
    auto next = poseRequest();
    next.pose_delta.emplace().torso_translation.x = 0.02;
    ASSERT_TRUE(controller->setGait(next));
    advance(30);
    advance(50);
    EXPECT_TRUE(controller->motionStopped());
    EXPECT_NEAR(bodyPose(*model).torso_pose.position.x, origin.torso_pose.position.x, 1e-9);
    auto absolute = poseRequest();
    absolute.body_pose.has_head_orientation = true;
    ASSERT_TRUE(controller->setGait(absolute));
    advance(50);
    EXPECT_NEAR(bodyPose(*model).torso_pose.position.x, origin.torso_pose.position.x, 1e-9);
}

TEST_F(SinglePoseTargetTest, HeadOnlyPreservesTorsoAndToesThroughoutTrajectory) {
    const auto origin = bodyPose(*model);
    auto request = poseRequest();
    request.body_pose.has_head_orientation = true;
    request.body_pose.head_orientation.yaw = -12;
    ASSERT_TRUE(controller->setGait(request));
    EXPECT_EQ(bodyPose(*model), origin);
    advance(25);
    const auto middle = bodyPose(*model);
    EXPECT_EQ(middle.torso_pose, origin.torso_pose);
    EXPECT_EQ(middle.toe_positions, origin.toe_positions);
    EXPECT_NEAR(middle.head_orientation.yaw, -2, 1e-9);
    advance(25);
    EXPECT_TRUE(controller->motionStopped());
    EXPECT_EQ(bodyPose(*model), resolveBodyPose(origin, request.body_pose));
}

TEST_F(SinglePoseTargetTest, ToesInterpolateWithTheSameTimingAsHeadAndTorso) {
    const auto origin = bodyPose(*model);
    auto request = poseRequest();
    request.body_pose = origin;
    request.body_pose.head_orientation.yaw += 10;
    request.body_pose.torso_pose.position.z += 0.004;
    for (auto& toe : request.body_pose.toe_positions) toe.z += 0.006;
    ASSERT_TRUE(controller->setGait(request));
    advance(25);
    const auto middle = bodyPose(*model);
    EXPECT_NEAR(middle.head_orientation.yaw, origin.head_orientation.yaw + 5, 1e-9);
    EXPECT_NEAR(middle.torso_pose.position.z, origin.torso_pose.position.z + 0.002, 1e-9);
    for (size_t i = 0; i < 6; ++i)
        EXPECT_NEAR(middle.toe_positions[i].z, origin.toe_positions[i].z + 0.003, 1e-9);
    advance(25);
    EXPECT_EQ(bodyPose(*model), request.body_pose);
    // An explicit repeated full pose still holds for the requested time.
    ASSERT_TRUE(controller->setGait(request));
    EXPECT_FALSE(controller->motionStopped());
    advance(49);
    EXPECT_FALSE(controller->motionStopped());
    advance(1);
    EXPECT_TRUE(controller->motionStopped());
}

TEST_F(SinglePoseTargetTest, PendingSameTypeResolvesOmissionsAtThePreviousEndpoint) {
    auto first = poseRequest();
    first.body_pose.has_torso_pose = true;
    first.body_pose.torso_pose.position.z = 0.007;
    ASSERT_TRUE(controller->setGait(first));
    advance(10);
    auto second = poseRequest();
    second.body_pose.has_head_orientation = true;
    second.body_pose.head_orientation.yaw = -15;
    ASSERT_TRUE(controller->setGait(second));
    EXPECT_TRUE(controller->hasPendingGait());
    advance(40);
    const auto endpoint = bodyPose(*model);
    EXPECT_EQ(endpoint.torso_pose, first.body_pose.torso_pose);
    advance(50);
    EXPECT_FALSE(controller->hasPendingGait());
    EXPECT_TRUE(controller->motionStopped());
    EXPECT_EQ(bodyPose(*model), resolveBodyPose(endpoint, second.body_pose));
}

TEST_F(SinglePoseTargetTest, CancellationFinishesCurrentTargetAndClearsPendingOnReplacement) {
    auto first = poseRequest();
    first.body_pose.has_head_orientation = true;
    first.body_pose.head_orientation.yaw = 15;
    const auto expected = resolveBodyPose(bodyPose(*model), first.body_pose);
    ASSERT_TRUE(controller->setGait(first));
    advance(10);
    auto pending = poseRequest();
    pending.body_pose.has_torso_pose = true;
    pending.body_pose.torso_pose.position.z = 0.02;
    ASSERT_TRUE(controller->setGait(pending));
    ASSERT_TRUE(controller->hasPendingGait());
    controller->requestStopSelectedGait();
    EXPECT_FALSE(controller->hasPendingGait());
    advance(90);
    EXPECT_TRUE(controller->motionStopped());
    EXPECT_EQ(bodyPose(*model), expected);
    first.body_pose.head_orientation.yaw = -15;
    ASSERT_TRUE(controller->setGait(first));
    advance(50);
    EXPECT_DOUBLE_EQ(bodyPose(*model).head_orientation.yaw, -15);
}
}  // namespace
