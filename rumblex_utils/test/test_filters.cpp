#include <gtest/gtest.h>

#include <rumblex_utils/filters.hpp>

TEST(LowPassFilterTest, AlphaOnePassThrough) {
    double last = 1.0;
    double target = 5.0;
    double alpha = 1.0;  // full weight target
    double filtered = utils::lowPassFilter(last, target, alpha);
    EXPECT_DOUBLE_EQ(filtered, target);
}

TEST(LowPassFilterTest, AlphaZeroHoldLast) {
    double last = 2.5;
    double target = 9.0;
    double alpha = 0.0;  // full weight last
    double filtered = utils::lowPassFilter(last, target, alpha);
    EXPECT_DOUBLE_EQ(filtered, last);
}

TEST(LowPassFilterTest, MidAlphaBlend) {
    double last = 2.0;
    double target = 6.0;
    double alpha = 0.25;  // expect 0.25*6 + 0.75*2 = 1.5 + 1.5 = 3.0
    double filtered = utils::lowPassFilter(last, target, alpha);
    EXPECT_DOUBLE_EQ(filtered, 3.0);
}

TEST(LowPassFilterTest, ConvergesTowardTarget) {
    double value = 0.0;
    double target = 10.0;
    double alpha = 0.2;
    for (int i = 0; i < 25; ++i) {
        value = utils::lowPassFilter(value, target, alpha);
    }
    // After enough iterations should be close but not equal
    EXPECT_NEAR(value, 10.0, 0.5);
    EXPECT_LT(value, 10.0);  // still below target due to exponential approach
}

TEST(ChangeRateLimitTest, LimitsIncrease) {
    double last = 0.0;
    double target = 10.0;
    double changeRate = 2.0;  // max delta
    double limited = utils::limitChangeRate(last, target, changeRate);
    EXPECT_DOUBLE_EQ(limited, last + changeRate);
}

TEST(ChangeRateLimitTest, LimitsDecrease) {
    double last = 10.0;
    double target = -5.0;
    double changeRate = 3.0;  // max delta
    double limited = utils::limitChangeRate(last, target, changeRate);
    EXPECT_DOUBLE_EQ(limited, last - changeRate);
}

TEST(ChangeRateLimitTest, WithinRatePassesThrough) {
    double last = 5.0;
    double target = 6.5;      // delta 1.5
    double changeRate = 2.0;  // max delta greater than needed
    double limited = utils::limitChangeRate(last, target, changeRate);
    EXPECT_DOUBLE_EQ(limited, target);
}

TEST(ChangeRateLimitUpTest, PositiveTargetPositiveLast) {
    double last = 5.0;
    double target = 10.0;
    double changeRate = 2.0;  // max delta
    double limited = utils::limitChangeRateUp(last, target, changeRate);
    EXPECT_DOUBLE_EQ(limited, last + changeRate);  // Should be limited to 7.0
}

TEST(ChangeRateLimitUpTest, NegativeTargetNegativeLast) {
    double last = -5.0;
    double target = -10.0;
    double changeRate = 2.0;  // max delta
    double limited = utils::limitChangeRateUp(last, target, changeRate);
    EXPECT_DOUBLE_EQ(limited, last - changeRate);  // Should be limited to -7.0
}

TEST(ChangeRateLimitUpTest, PositiveTargetNegativeLast) {
    double last = -5.0;
    double target = 3.0;
    double changeRate = 2.0;  // max delta
    double limited = utils::limitChangeRateUp(last, target, changeRate);
    EXPECT_DOUBLE_EQ(limited, last + changeRate);  // Should be limited to -3.0
}

TEST(KalmanFilterTest, ConvergesToMeasurement) {
    utils::CKalmanFilter<double> filter(0.05, 0.5, 1.0, 0.0);
    constexpr double measurement = 4.5;
    double estimate = 0.0;
    for (int i = 0; i < 10; ++i) {
        estimate = filter.step(measurement);
    }
    EXPECT_NEAR(estimate, measurement, 0.1);
    EXPECT_LT(filter.getUncertainty(), 0.5);
}

TEST(KalmanFilterTest, AppliesControlInputOnPredict) {
    utils::CKalmanFilter<double> filter(0.01, 0.5, 0.1, 1.0);
    double predicted = filter.predict(2.0, 0.5);  // expect +1.0
    EXPECT_DOUBLE_EQ(predicted, 2.0);
    // No measurement update, so state should hold prediction
    EXPECT_DOUBLE_EQ(filter.getState(), 2.0);
}

TEST(KalmanFilterTest, ResetChangesStateAndUncertainty) {
    utils::CKalmanFilter<double> filter(0.1, 0.5, 1.0, 0.0);
    filter.reset(3.0, 0.2);
    EXPECT_DOUBLE_EQ(filter.getState(), 3.0);
    EXPECT_DOUBLE_EQ(filter.getUncertainty(), 0.2);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
