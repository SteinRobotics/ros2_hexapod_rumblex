#include <gtest/gtest.h>

#include <thread>

#include "handler/face_preview.hpp"

namespace {
using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using mp_units::angular::unit_symbols::deg;
using namespace rumblex_geometry;
using namespace std::chrono_literals;

class FacePreviewTest : public ::testing::Test {
   protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        rclcpp::NodeOptions options;
        options.parameter_overrides({{"face_tracking.desktop_preview", true},
                                     {"face_tracking.max_yaw_deg", 30.0},
                                     {"face_tracking.max_pitch_deg", 20.0},
                                     {"face_tracking.max_speed_deg_s", 25.0},
                                     {"face_tracking.acceleration_deg_s2", 60.0},
                                     {"face_tracking.gain", 3.0},
                                     {"face_tracking.deadband_deg", 0.5}});
        node = std::make_shared<rclcpp::Node>("face_preview_test", options);
        node->declare_parameter("gait.generic.head_max_yaw_deg", 30.0);
        node->declare_parameter("gait.generic.head_max_pitch_deg", 20.0);
        preview = std::make_unique<brain::FacePreview>(node);
        publisher = node->create_publisher<rumblex_interfaces::msg::FaceTarget>("perception/face_target", 1);
        executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        executor->add_node(node);
    }
    void TearDown() override {
        executor->remove_node(node);
        executor.reset();
        preview.reset();
        node.reset();
        rclcpp::shutdown();
    }
    void observe(double x, double y, double age = 0,
                 const std::string& frame = "desktop_camera_optical_frame") {
        rumblex_interfaces::msg::FaceTarget target;
        target.valid = true;
        target.confidence = 0.95;
        target.header.frame_id = frame;
        target.header.stamp = node->now() - rclcpp::Duration::from_seconds(age);
        target.x_error = x;
        target.y_error = y;
        publisher->publish(target);
        std::this_thread::sleep_for(5ms);
        executor->spin_some();
    }
    std::shared_ptr<rclcpp::Node> node;
    std::unique_ptr<brain::FacePreview> preview;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
    rclcpp::Publisher<rumblex_interfaces::msg::FaceTarget>::SharedPtr publisher;
};

TEST_F(FacePreviewTest, FixedCameraFaceConvergesWithoutAccumulatingToLimit) {
    COrientation pose;
    for (int i = 0; i < 160; ++i) {
        observe(0.2, -0.1);
        const auto next = preview->update(pose, 0.02 * s, true);
        ASSERT_TRUE(next);
        EXPECT_LE(abs(next->yaw - pose.yaw).numerical_value_in(deg), 0.5 + 1e-9);
        EXPECT_LE(abs(next->pitch - pose.pitch).numerical_value_in(deg), 0.5 + 1e-9);
        pose = *next;
    }
    EXPECT_NEAR(pose.yaw.numerical_value_in(deg), -11.31, 0.6);
    EXPECT_NEAR(pose.pitch.numerical_value_in(deg), -5.60, 0.6);
}
TEST_F(FacePreviewTest, MissingStaleWrongFrameAndManualOverrideCannotMoveHead) {
    EXPECT_FALSE(preview->update(COrientation(), 0.02 * s, true));
    observe(0.2, 0.1, 1.0);
    EXPECT_FALSE(preview->update(COrientation(), 0.02 * s, true));
    observe(0.2, 0.1, 0, "camera_optical_frame");
    EXPECT_FALSE(preview->update(COrientation(), 0.02 * s, true));
    observe(0.2, 0.1);
    EXPECT_FALSE(preview->update(COrientation(), 0.02 * s, false));
    EXPECT_TRUE(preview->update(COrientation(), 0.02 * s, true));
    preview->suspend();
    observe(0.2, 0.1);
    EXPECT_FALSE(preview->active());
    EXPECT_FALSE(preview->update(COrientation(), 0.02 * s, true));
}
TEST_F(FacePreviewTest, SaturatesWithinConfiguredLimitsAndStopsAfterTimeout) {
    COrientation pose;
    for (int i = 0; i < 200; ++i) {
        observe(10, 10);
        const auto next = preview->update(pose, 0.02 * s, true);
        ASSERT_TRUE(next);
        pose = *next;
        EXPECT_LE(abs(pose.yaw).numerical_value_in(deg), 30.0);
        EXPECT_LE(abs(pose.pitch).numerical_value_in(deg), 20.0);
    }
    EXPECT_NEAR(pose.yaw.numerical_value_in(deg), -30.0, 0.6);
    EXPECT_NEAR(pose.pitch.numerical_value_in(deg), 20.0, 0.6);
    std::this_thread::sleep_for(320ms);
    EXPECT_FALSE(preview->update(pose, 0.02 * s, true));
}
}  // namespace
