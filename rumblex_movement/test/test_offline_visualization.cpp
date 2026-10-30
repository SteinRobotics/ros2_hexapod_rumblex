#include "offline_visualization.hpp"
#include "test_helpers.hpp"
#include "yaml-cpp/yaml.h"

namespace {
class OfflineVisualizationTest : public ::testing::TestWithParam<std::string> {
   protected:
    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
    }
    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }
};

TEST_P(OfflineVisualizationTest, RotatedTorsoKeepsToeTargetsFixed) {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--params-file",
                       std::string(RUMBLEX_ANATOMY_DIR) + "/" + GetParam() + "/anatomy.yaml"});
    auto node = std::make_shared<rclcpp::Node>("test_offline_visualization", options);
    rumblex_geometry::CBodyModel model(node);
    model.moveTorso(model.getStandingToePositions());
    auto pose = rumblex_geometry::bodyPose(model);
    pose.torso_pose.position.x = 0.002;
    pose.torso_pose.position.y = -0.003;
    pose.torso_pose.position.z = 0.001;
    pose.torso_pose.orientation.roll = 3.0;
    pose.torso_pose.orientation.pitch = -2.0;
    pose.torso_pose.orientation.yaw = 4.0;
    pose.head_orientation.yaw = 15.0;
    pose.head_orientation.pitch = -10.0;
    const auto joints = rumblex_movement::visualizationJoints(model, pose);
    ASSERT_EQ(joints.name.size(), 20u);
    EXPECT_NEAR(joints.position[18], utils::deg2rad(15), 1e-12);
    EXPECT_NEAR(joints.position[19], utils::deg2rad(-10), 1e-12);
    const rumblex_geometry::CPose torso(pose.torso_pose);
    for (size_t i = 0; i < 6; ++i) {
        auto index = rumblex_geometry::bodyLegOrder[i];
        // Forward kinematics independently reconstructs what RViz will display.
        model.setLegAngles(index, model.getLegAngles(index));
        auto toe = model.getToePositions().at(index);
        const double roll = utils::deg2rad(3), pitch = utils::deg2rad(-2), yaw = utils::deg2rad(4);
        const auto y = std::cos(roll) * toe.y - std::sin(roll) * toe.z;
        const auto z = std::sin(roll) * toe.y + std::cos(roll) * toe.z;
        const auto x = std::cos(pitch) * toe.x + std::sin(pitch) * z;
        rumblex_geometry::CPosition world_toe(
            std::cos(yaw) * x - std::sin(yaw) * y + torso.position.x,
            std::sin(yaw) * x + std::cos(yaw) * y + torso.position.y,
            -std::sin(pitch) * toe.x + std::cos(pitch) * z + torso.position.z);
        rumblex_movement::test_helpers::expectPositionNear(
            rumblex_geometry::CPosition(pose.toe_positions[i].x, pose.toe_positions[i].y,
                                        pose.toe_positions[i].z),
            world_toe, "toe " + std::to_string(i), 1e-9);
    }
}
TEST_P(OfflineVisualizationTest, DancesKeepEveryInterpolatedFootReachable) {
    for (const auto* dance : {"snippets/dance_hot_dogs_for_breakfast.yaml", "dance_dancing_street.yaml"}) {
        SCOPED_TRACE(dance);
        rclcpp::NodeOptions options;
        options.arguments({"--ros-args", "--params-file",
                           std::string(RUMBLEX_ANATOMY_DIR) + "/" + GetParam() + "/anatomy.yaml"});
        auto node = std::make_shared<rclcpp::Node>("test_dance_workspace", options);
        rumblex_geometry::CBodyModel model(node);
        model.moveTorso(model.getStandingToePositions());
        auto pose = rumblex_geometry::bodyPose(model);
        rumblex_geometry::CPose previous;
        const auto actions = YAML::LoadFile(std::string(RUMBLEX_SEQUENCE_DIR) + "/" + dance)["actions"];
        for (size_t action = 0; action < actions.size(); ++action) {
            const auto body = actions[action]["body_pose"];
            if (!body) continue;
            const auto position = body["torso_pose"]["position"];
            const auto orientation = body["torso_pose"]["orientation"];
            const rumblex_geometry::CPose target(position["x"].as<double>(), position["y"].as<double>(),
                                                 position["z"].as<double>(), orientation["roll"].as<double>(),
                                                 orientation["pitch"].as<double>(),
                                                 orientation["yaw"].as<double>());
            // Sampling the complete path also catches an unreachable intermediate pose.
            for (int step = 0; step <= 100; ++step) {
                SCOPED_TRACE(GetParam() + " action " + std::to_string(action) + " step " +
                             std::to_string(step));
                pose.torso_pose = previous.linearInterpolate(target, step / 100.0).toMsg();
                rumblex_movement::visualizationJoints(model, pose);
                const auto expected = model.getToePositions();
                for (const auto index : rumblex_geometry::bodyLegOrder) {
                    model.setLegAngles(index, model.getLegAngles(index));
                    // A clamped IK solution cannot reconstruct the requested foot position.
                    expectPositionNear(expected.at(index), model.getToePositions().at(index),
                                       "dance foot target cannot be reached", 1e-9);
                }
            }
            previous = target;
        }
    }
}

INSTANTIATE_TEST_SUITE_P(Robots, OfflineVisualizationTest, ::testing::Values("nox", "nira"));
}  // namespace
