/*******************************************************************************
 * Copyright (c) 2025 Christian Stein
 * ****************************************************************************/

#pragma once
/**
 * Simple PID Controller (header-only)
 * -----------------------------------
 * Features:
 *  - Configurable gains (Kp, Ki, Kd)
 *  - Optional output clamping (min/max)
 *  - Optional integral (windup) clamping
 *  - Update method taking: setpoint, measurement, dt
 *  - Reset for integral state & derivative history
 *  - Supports disabling integral or derivative (set Ki / Kd to 0)
 *  - Guards against dt <= 0.0 to avoid spikes
 *
 * Typical usage:
 *  CPIDController pid(1.0, 0.1, 0.01);
 *  pid.setOutputLimits(-10.0, 10.0);
 *  pid.setIntegralLimits(-2.0, 2.0);
 *  double u = pid.update(setpoint, measurement, dt_seconds);
 *
 * Notes:
 *  - Derivative term is based on error difference; for noisy signals consider
 *    adding external filtering of the measurement.
 *  - All math uses double. If you need float, create a templated variant.
 */

#include <algorithm>
#include <limits>

namespace utils {

class CPIDController {
   public:
    CPIDController(double kp = 0.0, double ki = 0.0, double kd = 0.0) : kp_(kp), ki_(ki), kd_(kd) {
    }

    void setGains(double kp, double ki, double kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    void setOutputLimits(double min_out, double max_out) {
        minOutput_ = std::min(min_out, max_out);
        maxOutput_ = std::max(min_out, max_out);
        has_output_limits_ = true;
    }

    void clearOutputLimits() {
        has_output_limits_ = false;
        minOutput_ = -std::numeric_limits<double>::infinity();
        maxOutput_ = std::numeric_limits<double>::infinity();
    }

    void setIntegralLimits(double min_int, double max_int) {
        minIntegral_ = std::min(min_int, max_int);
        maxIntegral_ = std::max(min_int, max_int);
        has_integral_limits_ = true;
    }

    void clearIntegralLimits() {
        has_integral_limits_ = false;
        minIntegral_ = -std::numeric_limits<double>::infinity();
        maxIntegral_ = std::numeric_limits<double>::infinity();
    }

    void reset() {
        integral_ = 0.0;
        prev_error_ = 0.0;
        first_update_ = true;
    }

    double getIntegral() const {
        return integral_;
    }
    double getLastError() const {
        return prev_error_;
    }

    double update(double setpoint, double measurement, double dt) {
        double error = setpoint - measurement;

        // Proportional
        double p = kp_ * error;

        // Integral (only accumulate if Ki != 0)
        if (ki_ != 0.0 && dt > 0.0) {
            integral_ += error * dt;
            if (has_integral_limits_) {
                integral_ = std::clamp(integral_, minIntegral_, maxIntegral_);
            }
        }
        double i = ki_ * integral_;

        // Derivative (only if Kd != 0 and not first update)
        double d = 0.0;
        if (!first_update_ && kd_ != 0.0 && dt > 0.0) {
            double derivative = (error - prev_error_) / dt;
            d = kd_ * derivative;
        }

        prev_error_ = error;
        first_update_ = false;

        double output = p + i + d;
        if (has_output_limits_) {
            output = std::clamp(output, minOutput_, maxOutput_);
        }
        return output;
    }

   private:
    // Gains
    double kp_ = 0.0;
    double ki_ = 0.0;
    double kd_ = 0.0;

    // State
    double integral_ = 0.0;
    double prev_error_ = 0.0;
    bool first_update_ = true;

    // Limits
    bool has_output_limits_ = false;
    double minOutput_ = -std::numeric_limits<double>::infinity();
    double maxOutput_ = std::numeric_limits<double>::infinity();

    bool has_integral_limits_ = false;
    double minIntegral_ = -std::numeric_limits<double>::infinity();
    double maxIntegral_ = std::numeric_limits<double>::infinity();
};

}  // namespace utils

// Backward-compat alias
using CPIDController = utils::CPIDController;
