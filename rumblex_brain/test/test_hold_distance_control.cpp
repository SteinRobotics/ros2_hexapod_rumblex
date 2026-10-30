#include <limits>

#include "feedback_test_fixture.hpp"
#include "gait/hold_distance_control.hpp"

namespace {
const HoldDistanceSettings settings{0.2, 0.02, 0.02, 0.02};
class HoldDistanceControlTest : public FeedbackTestFixture {
   protected:
    void SetUp() override {
        FeedbackTestFixture::SetUp();
        create(test_helpers::holdDistanceParameters());
        sensor = std::make_unique<RangeFeedback>(node);
    }
    void TearDown() override {
        sensor.reset();
        FeedbackTestFixture::TearDown();
    }
    std::unique_ptr<RangeFeedback> sensor;
};
TEST(HoldDistanceControl, RespectsSpeedAccelerationLimitsAndDeadband) {
    HoldDistanceLaw control(settings);
    EXPECT_NEAR(control.update(1.0, 0.6, 0.02), 0.0004, 1e-12);
    double speed = 0;
    for (int i = 0; i < 1000; ++i) speed = control.update(1.0, 0.6, 0.02);
    EXPECT_DOUBLE_EQ(speed, settings.max_speed);
    for (int i = 0; i < 100; ++i) speed = control.update(0.2, 0.6, 0.02);
    EXPECT_DOUBLE_EQ(speed, -settings.max_speed);
    for (int i = 0; i < 100; ++i) speed = control.update(0.61, 0.6, 0.02);
    EXPECT_DOUBLE_EQ(speed, 0.0);
}
TEST(HoldDistanceControl, RejectsInvalidInputs) {
    HoldDistanceLaw control(settings);
    for (double dt : {0.0, -0.02, 0.51, std::numeric_limits<double>::quiet_NaN()})
        EXPECT_THROW(control.update(1, 0.6, dt), std::invalid_argument);
    EXPECT_THROW(control.update(std::numeric_limits<double>::infinity(), 0.6, 0.02), std::invalid_argument);
}
TEST(HoldDistanceControl, SyntheticDistancePlantConverges) {
    HoldDistanceLaw control(settings);
    double range = 1.0;
    for (int i = 0; i < 4000; ++i) range -= control.update(range, 0.6, 0.02) * 0.02;
    EXPECT_NEAR(range, 0.6, 0.021);
}
TEST_F(HoldDistanceControlTest, DistanceRequiresOnlyRangeAndRejectsInvalidOrMisalignedReadings) {
    EXPECT_FALSE(sensor->range());
    publishRange();
    ASSERT_TRUE(sensor->range());
    EXPECT_NEAR(*sensor->range(), 0.8, 1e-6);
    EXPECT_EQ(node->count_subscribers("test_imu"), 0u);
    range.range = std::numeric_limits<float>::infinity();
    range_pub->publish(range);
    spin();
    EXPECT_FALSE(sensor->range());
    publishRange();
    range.header.frame_id = "rotated_sensor";
    range_pub->publish(range);
    spin();
    EXPECT_FALSE(sensor->range());
    publishRange();
    range.header.stamp = node->now() - rclcpp::Duration::from_seconds(2);
    range_pub->publish(range);
    spin();
    EXPECT_FALSE(sensor->range());
}
}  // namespace
