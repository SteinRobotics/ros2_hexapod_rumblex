#include <gtest/gtest.h>

#include "gait/gait_controller.hpp"
#include "parser/sequence_parser.hpp"
#include "rumblex_utils/body_pose.hpp"
#include "test_helpers.hpp"

namespace {
using namespace brain;
using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using mp_units::angular::unit_symbols::deg;

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

TEST(Interpolation, CurvesHaveExpectedSamplesAndEndpointSlopes) {
    const std::array types = {InterpolationType::Linear, InterpolationType::SinSquared,
                              InterpolationType::Sin, InterpolationType::Cos};
    const std::array quarter = {0.25, 0.1464466094067262, 0.0761204674887133, 0.3826834323650898};
    const std::array start_slope = {1.0, 0.0, 0.0, std::numbers::pi / 2.0};
    const std::array end_slope = {1.0, 0.0, std::numbers::pi / 2.0, 0.0};
    for (size_t i = 0; i < types.size(); ++i) {
        const auto curve = [type = types[i]](double t) { return interpolationProgress(t, type); };
        EXPECT_DOUBLE_EQ(curve(-1), 0);
        EXPECT_DOUBLE_EQ(curve(0), 0);
        EXPECT_DOUBLE_EQ(curve(1), 1);
        EXPECT_DOUBLE_EQ(curve(2), 1);
        EXPECT_NEAR(curve(0.25), quarter[i], 1e-12);
        constexpr double h = 1e-6;
        EXPECT_NEAR(curve(h) / h, start_slope[i], 1e-5);
        EXPECT_NEAR((1 - curve(1 - h)) / h, end_slope[i], 1e-5);
        for (int sample = 1; sample <= 100; ++sample)
            EXPECT_GE(curve(sample / 100.0), curve((sample - 1) / 100.0));
    }
}

TEST_F(SinglePoseTargetTest, AbsolutePoseHonorsShortDurationsAndClampsBelowFiveSamples) {
    EXPECT_EQ(MovementRequest().duration_s, 1.0 * s);
    for (const double seconds : {0.0, 0.05, 0.1, 0.5, 1.0}) {
        SCOPED_TRACE(seconds);
        const double origin = model->getHeadOrientation().yaw.numerical_value_in(deg);
        auto request = poseRequest();
        request.duration_s = seconds * s;
        request.interpolation = InterpolationType::Linear;
        request.body_pose.has_head_orientation = true;
        request.body_pose.head_orientation.yaw = origin + 1;
        ASSERT_TRUE(controller->setGait(request));
        const int samples = std::lround(std::max(seconds, 0.1) / 0.02);
        advance(1);
        EXPECT_NEAR(model->getHeadOrientation().yaw.numerical_value_in(deg), origin + 1.0 / samples, 1e-9);
        advance(samples - 2);
        EXPECT_FALSE(controller->motionStopped());
        advance(1);
        EXPECT_TRUE(controller->motionStopped());
        EXPECT_NEAR(model->getHeadOrientation().yaw.numerical_value_in(deg), origin + 1, 1e-9);
    }
}

TEST_F(SinglePoseTargetTest, ShortJointPoseSavesAndRestoresAtRequestedTime) {
    const auto origin = bodyPose(*model);
    auto request = poseRequest();
    request.duration_s = 0.1 * s;
    request.save_pose = true;
    const auto angles = model->getLegAngles(ELegIndex::RightFront);
    request.joint_targets[0] = std::array<double, 3>{angles.torso_coxa.numerical_value_in(deg) + 1,
                                                     angles.coxa_femur.numerical_value_in(deg),
                                                     angles.femur_tibia.numerical_value_in(deg)};
    ASSERT_TRUE(controller->setGait(request));
    advance(4);
    EXPECT_FALSE(controller->motionStopped());
    advance(1);
    EXPECT_TRUE(controller->motionStopped());
    EXPECT_NEAR(model->getLegAngles(ELegIndex::RightFront).torso_coxa.numerical_value_in(deg),
                angles.torso_coxa.numerical_value_in(deg) + 1, 1e-9);
    auto restore = poseRequest();
    restore.duration_s = 0.1 * s;
    restore.restore_pose = true;
    ASSERT_TRUE(controller->setGait(restore));
    advance(4);
    EXPECT_FALSE(controller->motionStopped());
    advance(1);
    EXPECT_TRUE(controller->motionStopped());
    EXPECT_EQ(bodyPose(*model), origin);
}

TEST_F(SinglePoseTargetTest, AllCurvesSynchronizeComponentsAndReachExactTargets) {
    const std::array types = {InterpolationType::Linear, InterpolationType::SinSquared,
                              InterpolationType::Sin, InterpolationType::Cos};
    const std::array progress = {0.25, 0.1464466094067262, 0.0761204674887133, 0.3826834323650898};
    for (size_t i = 0; i < types.size(); ++i) {
        const auto origin = bodyPose(*model);
        auto request = poseRequest();
        request.duration_s = 2.0 * s;
        request.interpolation = types[i];
        request.body_pose = origin;
        request.body_pose.head_orientation.yaw += 10;
        request.body_pose.torso_pose.position.z += 0.002;
        for (auto& toe : request.body_pose.toe_positions) toe.z += 0.003;
        ASSERT_TRUE(controller->setGait(request));
        advance(25);
        const auto actual = bodyPose(*model);
        EXPECT_NEAR(actual.head_orientation.yaw, origin.head_orientation.yaw + 10 * progress[i], 1e-9);
        EXPECT_NEAR(actual.torso_pose.position.z, origin.torso_pose.position.z + 0.002 * progress[i], 1e-9);
        for (size_t leg = 0; leg < 6; ++leg)
            EXPECT_NEAR(actual.toe_positions[leg].z, origin.toe_positions[leg].z + 0.003 * progress[i], 1e-9);
        advance(75);
        EXPECT_TRUE(controller->motionStopped());
        EXPECT_EQ(bodyPose(*model), request.body_pose);
    }
}

TEST_F(SinglePoseTargetTest, JointTargetsAndRestorationUseSelectedCurves) {
    const auto origin = bodyPose(*model);
    const auto initial_angles = model->getLegAngles(ELegIndex::RightFront);
    auto request = poseRequest();
    request.save_pose = true;
    request.interpolation = InterpolationType::Sin;
    request.joint_targets[0] = std::array<double, 3>{-20, 50, 60};
    ASSERT_TRUE(controller->setGait(request));
    advance(10);
    const double progress = 1 - std::cos(std::numbers::pi * 0.1);
    const auto actual_angles = model->getLegAngles(ELegIndex::RightFront);
    EXPECT_NEAR(actual_angles.coxa_femur.numerical_value_in(deg),
                initial_angles.coxa_femur.numerical_value_in(deg) * (1 - progress) + 50 * progress, 1e-9);
    EXPECT_NEAR(actual_angles.femur_tibia.numerical_value_in(deg),
                initial_angles.femur_tibia.numerical_value_in(deg) * (1 - progress) + 60 * progress, 1e-9);
    advance(40);
    const auto raised = bodyPose(*model);
    auto restore = poseRequest();
    restore.restore_pose = true;
    restore.interpolation = InterpolationType::Cos;
    ASSERT_TRUE(controller->setGait(restore));
    advance(10);
    EXPECT_NEAR(bodyPose(*model).toe_positions[0].z,
                raised.toe_positions[0].z + (origin.toe_positions[0].z - raised.toe_positions[0].z) *
                                                std::sin(std::numbers::pi * 0.1),
                1e-9);
    advance(40);
    EXPECT_EQ(bodyPose(*model), origin);
}

TEST_F(SinglePoseTargetTest, PendingRequestUsesItsOwnDefaultLinearCurve) {
    auto first = poseRequest();
    first.interpolation = InterpolationType::Sin;
    first.pose_delta.emplace().head_rotation.yaw = 10;
    ASSERT_TRUE(controller->setGait(first));
    advance(10);
    auto second = poseRequest();
    second.pose_delta.emplace().head_rotation.yaw = 20;
    ASSERT_TRUE(controller->setGait(second));
    advance(40);
    const auto origin = bodyPose(*model);
    advance(10);
    EXPECT_NEAR(bodyPose(*model).head_orientation.yaw, origin.head_orientation.yaw + 4, 1e-9);
    advance(40);
    EXPECT_TRUE(controller->motionStopped());
    EXPECT_NEAR(bodyPose(*model).head_orientation.yaw, origin.head_orientation.yaw + 20, 1e-9);
}

TEST_F(SinglePoseTargetTest, FlyingKeepsKneesStraightAndWingsAttachedThroughoutRolls) {
    CSequenceParser parser(node);
    ASSERT_TRUE(parser.parseFile(std::string(RUMBLEX_SEQUENCES_DIRECTORY) + "/flying.yaml"));
    const auto sequence = parser.getSequence("flying");
    ASSERT_TRUE(sequence);
    const auto& groups = sequence->get().actionGroups;
    ASSERT_EQ(groups.size(), 13u);
    const auto origin = bodyPose(*model);
    auto receiver = *model;
    double duration = 0;
    for (size_t i = 0; i < groups.size(); ++i) {
        SCOPED_TRACE(i);
        const auto action = std::dynamic_pointer_cast<RequestMovementType>(groups[i].at(0));
        ASSERT_TRUE(action);
        const auto& request = action->movementRequest;
        ASSERT_TRUE(controller->setGait(request));
        const double seconds = request.duration_s.numerical_value_in(s);
        duration += seconds;
        for (int tick = 0; tick < std::lround(seconds / 0.02); ++tick) {
            advance(1);
            const auto actual = bodyPose(*model);
            // Exercise the BodyPose -> IK boundary used by movement and visualization.
            receiver.moveTorso(toeTargets(actual), CPose(actual.torso_pose));
            for (size_t leg : {0u, 2u, 3u, 5u})
                EXPECT_EQ(actual.toe_positions[leg], origin.toe_positions[leg]);
            for (auto leg : {ELegIndex::RightMid, ELegIndex::LeftMid}) {
                const auto angles = receiver.getLegAngles(leg);
                if (i >= 1 && i <= 11) {
                    EXPECT_NEAR(angles.femur_tibia.numerical_value_in(deg), 90, 1e-5);
                }
                if (i >= 8 && i <= 11) {
                    EXPECT_NEAR(angles.torso_coxa.numerical_value_in(deg), 0, 1e-5);
                    EXPECT_NEAR(angles.coxa_femur.numerical_value_in(deg), 0, 1e-5);
                }
                // Reconstructed FK must reach the requested toe, not a clamped approximation.
                auto reconstructed = receiver;
                reconstructed.setLegAngles(leg, angles);
                const auto rebuilt = reconstructed.getToePositions().at(leg);
                const auto expected = model->getToePositions().at(leg);
                // At a straight knee, acos magnifies roundoff to nanometres.
                EXPECT_NEAR(rebuilt.x.numerical_value_in(m), expected.x.numerical_value_in(m), 1e-8);
                EXPECT_NEAR(rebuilt.y.numerical_value_in(m), expected.y.numerical_value_in(m), 1e-8);
                EXPECT_NEAR(rebuilt.z.numerical_value_in(m), expected.z.numerical_value_in(m), 1e-8);
            }
        }
        ASSERT_TRUE(controller->motionStopped());
        if (i >= 8 && i <= 11) {
            EXPECT_NEAR(model->getTorsoPose().orientation.roll.numerical_value_in(deg), i % 2 == 0 ? -6 : 6,
                        1e-9);
        }
    }
    EXPECT_DOUBLE_EQ(duration, 16.0);
    EXPECT_EQ(bodyPose(*model), origin);
}

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
                EXPECT_EQ(movement->movementRequest.interpolation, InterpolationType::SinSquared);
                const auto before = bodyPose(*model);
                ASSERT_TRUE(controller->setGait(movement->movementRequest));
                EXPECT_EQ(bodyPose(*model), before);
                double previous = before.head_orientation.yaw;
                double elapsed = 0;
                // Alternating intervals exercise elapsed-time sampling (1.5 seconds total).
                for (int tick = 0; tick < 100; ++tick) {
                    controller->updateSelectedGait(Velocity(), CPose(), COrientation(),
                                                   (tick % 2 ? 0.02 : 0.01) * s);
                    auto actual = bodyPose(*model);
                    elapsed += tick % 2 ? 0.02 : 0.01;
                    const double eased = std::pow(std::sin(std::numbers::pi * elapsed / 3.0), 2);
                    EXPECT_NEAR(actual.head_orientation.yaw,
                                before.head_orientation.yaw + (phase ? -sign : sign) * 25 * eased, 1e-9);
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

TEST_F(SinglePoseTargetTest, MetalDanceRestoresDisplacedEntrancePose) {
    CSequenceParser parser(node);
    ASSERT_TRUE(parser.parseFile(std::string(RUMBLEX_SEQUENCES_DIRECTORY) + "/dance_metal.yaml"));
    const auto sequence = parser.getSequence("dance_metal");
    ASSERT_TRUE(sequence);
    const auto origin = bodyPose(*model);
    for (const auto& group : sequence->get().actionGroups) {
        for (const auto& request : group) {
            const auto movement = std::dynamic_pointer_cast<RequestMovementType>(request);
            if (!movement) continue;
            ASSERT_TRUE(controller->setGait(movement->movementRequest));
            const auto seconds = movement->movementRequest.duration_s.numerical_value_in(s);
            advance(static_cast<int>(std::ceil(seconds / 0.02 - 1e-9)));
            EXPECT_TRUE(controller->motionStopped());
        }
    }
    EXPECT_EQ(bodyPose(*model), origin);
}

TEST_F(SinglePoseTargetTest, HotDogsAccentsFinishWithOriginalFeetAndNeutralBody) {
    CSequenceParser parser(node);
    ASSERT_TRUE(
        parser.parseFile(std::string(RUMBLEX_SEQUENCES_DIRECTORY) + "/dance_hot_dogs_for_breakfast.yaml"));
    const auto sequence = parser.getSequence("dance_hot_dogs_for_breakfast");
    ASSERT_TRUE(sequence);
    const auto toes = bodyPose(*model).toe_positions;
    for (const auto& group : sequence->get().actionGroups) {
        for (const auto& request : group) {
            const auto movement = std::dynamic_pointer_cast<RequestMovementType>(request);
            if (!movement) continue;
            ASSERT_TRUE(controller->setGait(movement->movementRequest));
            const auto seconds = movement->movementRequest.duration_s.numerical_value_in(s);
            advance(static_cast<int>(std::ceil(seconds / 0.02 - 1e-9)));
            EXPECT_TRUE(controller->motionStopped());
        }
    }
    const auto actual = bodyPose(*model);
    EXPECT_NEAR(actual.torso_pose.position.x, 0, 1e-9);
    EXPECT_NEAR(actual.torso_pose.position.y, 0, 1e-9);
    EXPECT_NEAR(actual.torso_pose.position.z, 0, 1e-9);
    EXPECT_NEAR(actual.torso_pose.orientation.roll, 0, 1e-9);
    EXPECT_NEAR(actual.torso_pose.orientation.pitch, 0, 1e-9);
    EXPECT_NEAR(actual.torso_pose.orientation.yaw, 0, 1e-9);
    EXPECT_NEAR(actual.head_orientation.pitch, 0, 1e-9);
    EXPECT_NEAR(actual.head_orientation.yaw, 0, 1e-9);
    for (size_t i = 0; i < toes.size(); ++i) {
        EXPECT_NEAR(actual.toe_positions[i].x, toes[i].x, 1e-9);
        EXPECT_NEAR(actual.toe_positions[i].y, toes[i].y, 1e-9);
        EXPECT_NEAR(actual.toe_positions[i].z, toes[i].z, 1e-9);
    }
}

TEST_F(SinglePoseTargetTest, BiteRestoresStartingPoseWithoutVoiceTrigger) {
    CSequenceParser parser(node);
    ASSERT_TRUE(parser.parseFile(std::string(RUMBLEX_SEQUENCES_DIRECTORY) + "/bite.yaml"));
    EXPECT_FALSE(parser.getSequenceForVoiceRequest("commandBite"));
    const auto sequence = parser.getSequence("bite");
    ASSERT_TRUE(sequence);
    const auto& groups = sequence->get().actionGroups;
    ASSERT_EQ(groups.size(), 6u);
    const auto origin = bodyPose(*model);
    const std::array durations = {0.22, 0.12, 0.18, 0.28, 0.35, 1.0};
    const std::array forward = {-0.008, -0.004, 0.022, -0.006, 0.0, 0.0};
    const std::array pitch = {-5.0, 5.0, 14.0, 4.0, 0.0, 0.0};
    for (size_t i = 0; i < groups.size(); ++i) {
        SCOPED_TRACE(i);
        ASSERT_EQ(groups[i].size(), 1u);
        const auto action = std::dynamic_pointer_cast<RequestMovementType>(groups[i][0]);
        ASSERT_TRUE(action);
        const auto& request = action->movementRequest;
        EXPECT_EQ(request.type, MovementRequest::SINGLE_POSE);
        EXPECT_EQ(request.duration_s, durations[i] * s);
        EXPECT_EQ(request.save_pose, i == 0);
        EXPECT_EQ(request.restore_pose, i == 5);
        EXPECT_EQ(request.interpolation, i == 1   ? InterpolationType::Sin
                                         : i == 2 ? InterpolationType::Cos
                                                  : InterpolationType::SinSquared);
        ASSERT_TRUE(controller->setGait(request));
        const int samples = static_cast<int>(std::ceil(durations[i] / 0.02 - 1e-9));
        for (int sample = 0; sample < samples; ++sample) {
            EXPECT_FALSE(controller->motionStopped());
            advance(1);
            EXPECT_EQ(bodyPose(*model).toe_positions, origin.toe_positions);
        }
        EXPECT_TRUE(controller->motionStopped());
        const auto actual = bodyPose(*model);
        EXPECT_NEAR(actual.torso_pose.position.x, origin.torso_pose.position.x + forward[i], 1e-9);
        EXPECT_NEAR(actual.head_orientation.pitch, origin.head_orientation.pitch + pitch[i], 1e-9);
        if (i == 4) {
            EXPECT_NEAR(actual.torso_pose.position.z, origin.torso_pose.position.z, 1e-9);
            EXPECT_NEAR(actual.torso_pose.orientation.pitch, origin.torso_pose.orientation.pitch, 1e-9);
        }
    }
    EXPECT_EQ(bodyPose(*model), origin);
}

TEST_F(SinglePoseTargetTest, StompAnnouncesLiftsStrikesAndRestoresRightFrontLeg) {
    CSequenceParser parser(node);
    ASSERT_TRUE(parser.parseFile(std::string(RUMBLEX_SEQUENCES_DIRECTORY) + "/stomp.yaml"));
    EXPECT_FALSE(parser.getSequenceForVoiceRequest("commandStomp"));
    const auto sequence = parser.getSequence("stomp");
    ASSERT_TRUE(sequence);
    const auto& groups = sequence->get().actionGroups;
    ASSERT_EQ(groups.size(), 3u);
    const auto origin = bodyPose(*model);
    const std::array durations = {0.5, 0.1, 1.0};
    const std::array offsets = {0.02, -0.02, 0.0};
    for (size_t i = 0; i < groups.size(); ++i) {
        SCOPED_TRACE(i);
        ASSERT_EQ(groups[i].size(), i == 0 ? 2u : 1u);
        std::shared_ptr<RequestMovementType> movement;
        std::shared_ptr<RequestTalking> speech;
        for (const auto& action : groups[i]) {
            if (auto request = std::dynamic_pointer_cast<RequestMovementType>(action)) movement = request;
            if (auto request = std::dynamic_pointer_cast<RequestTalking>(action)) speech = request;
        }
        if (i == 0) {
            ASSERT_TRUE(speech);
            EXPECT_EQ(speech->text, "Nein");
        }
        ASSERT_TRUE(movement);
        const auto& request = movement->movementRequest;
        EXPECT_EQ(request.type, MovementRequest::SINGLE_POSE);
        EXPECT_EQ(request.duration_s, durations[i] * s);
        EXPECT_EQ(request.save_pose, i == 0);
        EXPECT_EQ(request.restore_pose, i == 2);
        if (i < 2) {
            ASSERT_TRUE(request.pose_delta);
            EXPECT_DOUBLE_EQ(request.pose_delta->toe_translation[0].z, i == 0 ? 0.02 : -0.04);
        }
        ASSERT_TRUE(controller->setGait(request));
        const int samples = std::lround(durations[i] / 0.02);
        const double before = i == 0 ? 0.0 : offsets[i - 1];
        for (int tick = 1; tick <= samples; ++tick) {
            advance(1);
            EXPECT_EQ(controller->motionStopped(), tick == samples);
            const auto actual = bodyPose(*model);
            EXPECT_EQ(actual.head_orientation, origin.head_orientation);
            EXPECT_EQ(actual.torso_pose, origin.torso_pose);
            for (size_t leg = 1; leg < 6; ++leg)
                EXPECT_EQ(actual.toe_positions[leg], origin.toe_positions[leg]);
            EXPECT_DOUBLE_EQ(actual.toe_positions[0].x, origin.toe_positions[0].x);
            EXPECT_DOUBLE_EQ(actual.toe_positions[0].y, origin.toe_positions[0].y);
            EXPECT_NEAR(actual.toe_positions[0].z,
                        origin.toe_positions[0].z + before + (offsets[i] - before) * tick / samples, 1e-9);
        }
    }
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
        raised.moveTorso(raised.getToePositions(), CPose(0, 0, 0, 0, -8, 0));
        raised.setLegAngles(ELegIndex::RightFront, CLegAngles(-25, 50, 60));
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
            EXPECT_EQ(actual.torso_pose, expected.torso_pose);
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

TEST_F(SinglePoseTargetTest, ClapKeepsSupportPlantedAndLandsBeforeRecentering) {
    CSequenceParser parser(node);
    ASSERT_TRUE(parser.parseFile(std::string(RUMBLEX_SEQUENCES_DIRECTORY) + "/clap.yaml"));
    const auto sequence = parser.getSequence("clap");
    ASSERT_TRUE(sequence);
    ASSERT_EQ(sequence->get().actionGroups.size(), 22u);
    const auto origin = bodyPose(*model);
    double elapsed = 0;
    for (size_t i = 0; i < sequence->get().actionGroups.size(); ++i) {
        SCOPED_TRACE(i);
        const auto action =
            std::dynamic_pointer_cast<RequestMovementType>(sequence->get().actionGroups[i].at(0));
        ASSERT_TRUE(action);
        const auto& request = action->movementRequest;
        const bool contact = i >= 2 && i < 20 && (i - 2) % 3 == 1;
        const bool close = i >= 2 && i < 20 && (i - 2) % 3 == 0;
        EXPECT_EQ(request.pose_delta.has_value(), contact);
        EXPECT_EQ(request.interpolation, close ? InterpolationType::Cos : InterpolationType::SinSquared);
        if (contact) {
            EXPECT_EQ(request.duration_s, 0.12 * s);
        }
        if (close) {
            EXPECT_EQ(request.duration_s, (i == 2 ? 0.6 : 0.5) * s);
            const auto opening =
                std::dynamic_pointer_cast<RequestMovementType>(sequence->get().actionGroups[i + 2].at(0));
            ASSERT_TRUE(opening);
            EXPECT_GE(opening->movementRequest.duration_s, 2 * request.duration_s);
        }
        ASSERT_TRUE(controller->setGait(request));
        const int samples = std::lround(request.duration_s.numerical_value_in(s) / 0.02);
        for (int sample = 0; sample < samples; ++sample) {
            advance(1);
            const auto pose = bodyPose(*model);
            EXPECT_GE(pose.head_orientation.pitch, -4.0 - 1e-9);
            EXPECT_LE(pose.head_orientation.pitch, std::max(5.0, origin.head_orientation.pitch) + 1e-9);
            // Validate reachability throughout the trajectory, including landing.
            auto check = *model;
            for (const auto leg : bodyLegOrder) {
                const auto target = model->getToePositions().at(leg);
                check.setLegAngles(leg, model->getLegAngles(leg));
                test_helpers::expectPositionNear(target, check.getToePositions().at(leg), "clap", 1e-8);
            }
            for (const size_t leg : {1u, 2u, 4u, 5u})
                EXPECT_EQ(pose.toe_positions[leg], origin.toe_positions[leg]);
            if (i >= 1 && i <= 20) {
                EXPECT_NEAR(pose.torso_pose.position.x, -0.065, 1e-9);
                EXPECT_NEAR(pose.torso_pose.position.y, 0, 1e-9);
                EXPECT_GE(pose.torso_pose.position.z, -0.013 - 1e-9);
                EXPECT_LE(pose.torso_pose.position.z, -0.010 + 1e-9);
                EXPECT_NEAR(pose.torso_pose.orientation.roll, 0, 1e-9);
                EXPECT_NEAR(pose.torso_pose.orientation.pitch, 0, 1e-9);
                EXPECT_NEAR(pose.torso_pose.orientation.yaw, 0, 1e-9);
                // Geometric proxy: 3 cm forward of the torso, with at least
                // 3 cm clearance from EVERY edge of the four-foot polygon.
                // This is deliberately not a measured center-of-mass assertion.
                const std::array<size_t, 4> support = {1, 4, 5, 2};
                const double x = pose.torso_pose.position.x + 0.03;
                const double y = pose.torso_pose.position.y;
                for (size_t edge = 0; edge < support.size(); ++edge) {
                    const auto& a = pose.toe_positions[support[edge]];
                    const auto& b = pose.toe_positions[support[(edge + 1) % support.size()]];
                    const double dx = b.x - a.x;
                    const double dy = b.y - a.y;
                    EXPECT_GE((dx * (y - a.y) - dy * (x - a.x)) / std::hypot(dx, dy), 0.03);
                }
            }
            if (i == 21) {
                for (const size_t leg : {0u, 3u})
                    EXPECT_EQ(pose.toe_positions[leg], origin.toe_positions[leg]);
            }
        }
        EXPECT_TRUE(controller->motionStopped());
        if (i >= 2 && i < 20) {
            const double expected_pitch = close ? 3.0 : contact ? 5.0 : -4.0;
            EXPECT_NEAR(bodyPose(*model).head_orientation.pitch, expected_pitch, 1e-9);
        }
        elapsed += request.duration_s.numerical_value_in(s);
        if (i == 20) {
            for (const size_t leg : {0u, 3u})
                EXPECT_EQ(bodyPose(*model).toe_positions[leg], origin.toe_positions[leg]);
        }
    }
    EXPECT_NEAR(elapsed, 18.02, 1e-9);
    auto expected = origin;
    expected.torso_pose = rumblex_interfaces::msg::BodyPose().torso_pose;
    EXPECT_EQ(bodyPose(*model), expected);
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
