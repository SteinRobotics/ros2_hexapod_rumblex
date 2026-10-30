#include <limits>

#include "feedback_test_fixture.hpp"
#include "gait/level_torso_control.hpp"

namespace {
const LevelTorsoSettings settings{0.3, 0.017, 0.14, 0.035};
class LevelTorsoControlTest : public FeedbackTestFixture {
   protected:
    void SetUp() override {
        FeedbackTestFixture::SetUp();
        create(test_helpers::levelTorsoParameters());
        sensor = std::make_unique<ImuFeedback>(node);
    }
    void TearDown() override {
        sensor.reset();
        FeedbackTestFixture::TearDown();
    }
    std::unique_ptr<ImuFeedback> sensor;
};
TEST(LevelTorsoControl, RespectsTiltRateLimitsAndDeadband) {
    LevelTorsoLaw control(settings);
    control.reset({0, 0});
    auto tilt = control.update({0.2, -0.2}, 0.02);
    EXPECT_NEAR(tilt.roll, -0.0007, 1e-12);
    EXPECT_NEAR(tilt.pitch, 0.0007, 1e-12);
    for (int i = 0; i < 1000; ++i) tilt = control.update({0.2, -0.2}, 0.02);
    EXPECT_DOUBLE_EQ(tilt.roll, -settings.max_tilt);
    EXPECT_DOUBLE_EQ(tilt.pitch, settings.max_tilt);
    control.reset({0.03, -0.04});
    tilt = control.update({0.01, -0.01}, 0.02);
    EXPECT_DOUBLE_EQ(tilt.roll, 0.03);
    EXPECT_DOUBLE_EQ(tilt.pitch, -0.04);
}
TEST(LevelTorsoControl, RejectsInvalidInputs) {
    LevelTorsoLaw control(settings);
    for (double dt : {0.0, -0.02, 0.51, std::numeric_limits<double>::quiet_NaN()})
        EXPECT_THROW(control.update({0, 0}, dt), std::invalid_argument);
    EXPECT_THROW(control.update({std::numeric_limits<double>::quiet_NaN(), 0}, 0.02), std::invalid_argument);
    EXPECT_THROW(control.reset({1, 0}), std::invalid_argument);
}
TEST(LevelTorsoControl, SyntheticTiltPlantConverges) {
    LevelTorsoLaw control(settings);
    TorsoTilt measured{0.08, -0.05};
    for (int i = 0; i < 4000; ++i) {
        const auto command = control.update(measured, 0.02);
        measured = {0.08 + command.roll, -0.05 + command.pitch};
    }
    EXPECT_LT(std::abs(measured.roll), 0.018);
    EXPECT_LT(std::abs(measured.pitch), 0.018);
}
TEST_F(LevelTorsoControlTest, LevelingRequiresOnlyImuAndTransformsUprightMount) {
    EXPECT_FALSE(sensor->tilt());
    publishImu(0.1, -0.2);
    const auto tilt = sensor->tilt();
    ASSERT_TRUE(tilt);
    EXPECT_NEAR(tilt->roll, 0.1, 1e-6);
    EXPECT_NEAR(tilt->pitch, -0.2, 1e-6);
    EXPECT_EQ(node->count_subscribers("test_range"), 0u);
    imu.orientation_covariance[0] = -1;
    imu_pub->publish(imu);
    spin();
    EXPECT_FALSE(sensor->tilt());
    imu.orientation_covariance[0] = 0;
    publishImu();
    imu.orientation.w = 4;
    imu_pub->publish(imu);
    spin();
    EXPECT_FALSE(sensor->tilt());
    publishImu();
    imu.header.frame_id = "missing_frame";
    imu_pub->publish(imu);
    spin();
    EXPECT_FALSE(sensor->tilt());
    publishImu();
    imu.header.stamp = node->now() - rclcpp::Duration::from_seconds(2);
    imu_pub->publish(imu);
    spin();
    EXPECT_FALSE(sensor->tilt());
}
}  // namespace
