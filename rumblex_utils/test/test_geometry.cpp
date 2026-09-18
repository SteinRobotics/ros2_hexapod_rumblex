#include <gtest/gtest.h>

#include "rumblex_utils/geometry.hpp"

using namespace utils;

TEST(GeometryUtils, SinValueNearZero_ExactZero) {
    double phase = 0.0;
    double threshold = 0.0;
    EXPECT_TRUE(isSinValueNearZero(phase, threshold));
}

TEST(GeometryUtils, SinValueNearZero_EqualSmall) {
    double phase = 0.09;
    double threshold = 0.10;
    EXPECT_TRUE(isSinValueNearZero(phase, threshold));
}

TEST(GeometryUtils, SinValueNearZero_PhaseLargerThanThreshold) {
    double phase = 0.2;
    double threshold = 0.1;
    // sin(0.2) > sin(0.1) on (0, pi/2)
    EXPECT_FALSE(isSinValueNearZero(phase, threshold));
}

TEST(GeometryUtils, SinValueNearZero_NegativePhase) {
    double phase = -0.1;
    double threshold = 0.05;
    // abs(sin(-0.1)) > abs(sin(0.05))
    EXPECT_FALSE(isSinValueNearZero(phase, threshold));
}

TEST(GeometryUtils, SinValueNearZero_LargeThresholdAlwaysTrue) {
    double phase = 2.0;         // arbitrary
    double threshold = M_PI_2;  // sin(threshold) == 1
    EXPECT_TRUE(isSinValueNearZero(phase, threshold));
}

TEST(GeometryUtils, SinValueNearZero_nearPi) {
    double phase = M_PI - 0.05;
    double threshold = 0.1;
    EXPECT_TRUE(isSinValueNearZero(phase, threshold));
}

TEST(GeometryUtils, SinValueNearZero_near2Pi) {
    double phase = TWO_PI - 0.05;
    double threshold = 0.1;
    EXPECT_TRUE(isSinValueNearZero(phase, threshold));
}

TEST(GeometryUtils, SinValueNearZero_nearZero) {
    double phase = 0 + 0.05;
    double threshold = 0.1;
    EXPECT_TRUE(isSinValueNearZero(phase, threshold));
}

TEST(GeometryUtils, SinValueNearZero_myTest) {
    double phase = 5.83;
    double threshold = 0.4;
    EXPECT_FALSE(isSinValueNearZero(phase, threshold));
}
