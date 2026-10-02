#include "offline_visualization.hpp"
#include "test_helpers.hpp"

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
    pose.head_pose.yaw = 15.0;
    pose.head_pose.pitch = -10.0;
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
INSTANTIATE_TEST_SUITE_P(Robots, OfflineVisualizationTest, ::testing::Values("nox", "nira"));
}  // namespace
