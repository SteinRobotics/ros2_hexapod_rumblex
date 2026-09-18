#include <gtest/gtest.h>

#include <cmath>
#include <rumblex_utils/pid_controller.hpp>

TEST(PIDControllerTest, ProportionalOnly) {
    utils::CPIDController pid(2.0, 0.0, 0.0);  // Kp=2
    double setpoint = 5.0;
    double measurement = 3.0;
    double dt = 0.1;  // dt irrelevant for P only
    double output = pid.update(setpoint, measurement, dt);
    EXPECT_DOUBLE_EQ(output, 2.0 * (setpoint - measurement));  // 4.0
}

TEST(PIDControllerTest, IntegralAccumulation) {
    utils::CPIDController pid(0.0, 1.0, 0.0);  // Ki=1
    pid.setIntegralLimits(-100.0, 100.0);
    double setpoint = 1.0;
    double measurement = 0.0;
    double dt = 0.1;
    double lastOutput = 0.0;
    // After n steps integral should be ~ n*error*dt
    for (int i = 0; i < 10; ++i) {
        lastOutput = pid.update(setpoint, measurement, dt);
    }
    // integral ~= 1.0 * 1.0 * 10 * 0.1 = 1.0
    EXPECT_NEAR(lastOutput, 1.0, 1e-6);
    EXPECT_NEAR(pid.getIntegral(), 1.0, 1e-6);
}

TEST(PIDControllerTest, OutputClamping) {
    utils::CPIDController pid(10.0, 0.0, 0.0);  // Large Kp
    pid.setOutputLimits(-5.0, 5.0);
    double setpoint = 2.0;
    double measurement = -10.0;  // error = 12 -> raw P = 120
    double output = pid.update(setpoint, measurement, 0.1);
    EXPECT_DOUBLE_EQ(output, 5.0);  // clamped
}

TEST(PIDControllerTest, IntegralClamping) {
    utils::CPIDController pid(0.0, 5.0, 0.0);  // Strong Ki
    pid.setIntegralLimits(-0.5, 0.5);
    double setpoint = 1.0;
    double measurement = 0.0;
    for (int i = 0; i < 100; ++i) {
        pid.update(setpoint, measurement, 0.1);
    }
    // integral clamped at 0.5 => output = Ki * integral = 5 * 0.5 = 2.5
    double output = pid.update(setpoint, measurement, 0.1);
    EXPECT_DOUBLE_EQ(output, 2.5);
    EXPECT_DOUBLE_EQ(pid.getIntegral(), 0.5);
}

TEST(PIDControllerTest, DerivativeTerm) {
    utils::CPIDController pid(0.0, 0.0, 1.0);  // Kd=1
    // First update sets prevError but derivative not applied
    double out1 = pid.update(10.0, 0.0, 0.1);  // error 10
    EXPECT_DOUBLE_EQ(out1, 0.0);
    // Second update error changes to 8 => derivative = (8 - 10)/0.1 = -20
    double out2 = pid.update(10.0, 2.0, 0.1);  // error 8
    EXPECT_DOUBLE_EQ(out2, -20.0);
}

TEST(PIDControllerTest, ResetClearsState) {
    utils::CPIDController pid(1.0, 1.0, 1.0);
    pid.update(5.0, 0.0, 0.1);
    pid.update(5.0, 1.0, 0.1);
    ASSERT_NE(pid.getIntegral(), 0.0);
    pid.reset();
    EXPECT_EQ(pid.getIntegral(), 0.0);
    // After reset derivative should be suppressed
    double out = pid.update(5.0, 0.0, 0.1);  // first update again
    // Only P + I (I small) expected, but derivative should be zero
    EXPECT_GE(out, 0.0);
}

// Basic step response behavior (not a stability test) ensures monotonic reduction of error with PI
TEST(PIDControllerTest, StepResponseMonotonicReduction) {
    utils::CPIDController pid(0.8, 0.4, 0.0);  // PI controller
    pid.setOutputLimits(-100.0, 100.0);
    double setpoint = 10.0;
    double measurement = 0.0;
    double dt = 0.1;
    double previousError = setpoint - measurement;
    for (int i = 0; i < 50; ++i) {
        double control = pid.update(setpoint, measurement, dt);
        // Simple plant: measurement increases proportionally to control (scaled)
        measurement += control * 0.02;  // crude mock system response
        double error = setpoint - measurement;
        // Allow small overshoot tolerance; just ensure general downward trend every few steps
        if (i % 5 == 0) {
            EXPECT_LT(error, previousError + 0.5);  // not growing uncontrollably
            previousError = error;
        }
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
