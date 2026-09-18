#include <gtest/gtest.h>

#include <array>
#include <vector>

#include "rumblex_utils/linear_interpolation.hpp"

using namespace rumblex_utils;

TEST(LinearInterpolationTest, ScalarLerp) {
    double a = 0.0;
    double b = 10.0;
    EXPECT_DOUBLE_EQ(linearInterpolate(a, b, 0.0), 0.0);
    EXPECT_DOUBLE_EQ(linearInterpolate(a, b, 0.5), 5.0);
    EXPECT_DOUBLE_EQ(linearInterpolate(a, b, 1.0), 10.0);
}

TEST(LinearInterpolationTest, ScalarLerpClamped) {
    float a = 0.0f;
    float b = 100.0f;
    EXPECT_FLOAT_EQ(linearInterpolateClamped(a, b, -1.0), a);
    EXPECT_FLOAT_EQ(linearInterpolateClamped(a, b, 0.0), a);
    EXPECT_FLOAT_EQ(linearInterpolateClamped(a, b, 0.25), 25.0f);
    EXPECT_FLOAT_EQ(linearInterpolateClamped(a, b, 1.0), b);
    EXPECT_FLOAT_EQ(linearInterpolateClamped(a, b, 2.0), b);
}

TEST(LinearInterpolationTest, VectorLerpContainer) {
    std::vector<double> a{0.0, 10.0, 20.0};
    std::vector<double> b{10.0, 20.0, 30.0};
    auto out = linearInterpolateContainer(a, b, 0.5);
    ASSERT_EQ(out.size(), 3);
    EXPECT_DOUBLE_EQ(out[0], 5.0);
    EXPECT_DOUBLE_EQ(out[1], 15.0);
    EXPECT_DOUBLE_EQ(out[2], 25.0);
}

TEST(LinearInterpolationTest, ArrayLerpContainer) {
    std::array<float, 3> a{{0.0f, 5.0f, 10.0f}};
    std::array<float, 3> b{{10.0f, 15.0f, 20.0f}};
    auto out = linearInterpolateContainer(a, b, 0.25);
    EXPECT_FLOAT_EQ(out[0], 2.5f);
    EXPECT_FLOAT_EQ(out[1], 6.25f);
    EXPECT_FLOAT_EQ(out[2], 12.5f);
}

TEST(LinearInterpolationTest, ContainerClamped) {
    std::vector<int> a{0, 0, 0};
    std::vector<int> b{10, 20, 30};
    auto out0 = linearInterpolateContainerClamped(a, b, -0.5);
    EXPECT_EQ(out0, a);
    auto out1 = linearInterpolateContainerClamped(a, b, 1.5);
    EXPECT_EQ(out1, b);
}
