#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "gait/gait_interfaces.hpp"
#include "gait/gait_parameters.hpp"
#include "gait/pose_model.hpp"

namespace brain {

class CHighFiveGait : public ISequenceGait {
   public:
    CHighFiveGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> kinematics,
                  Parameters::HighFive& params);
    ~CHighFiveGait() override = default;

    void start(double duration_s, uint8_t direction) override;
    bool update() override;
    void requestStop() override;
    void cancelStop() override;
    EGaitState state() const override {
        return state_;
    }

   private:
    enum class EPhase { Idle, Raising, Holding, Lowering, Finished };

    void applyInterpolatedPose(double alpha);
    void transitionToLowering();

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CPoseModel> kinematics_;
    Parameters::HighFive params_;

    EGaitState state_ = EGaitState::Stopped;
    EPhase phase_ = EPhase::Idle;

    CLegAngles initial_leg_angles_{};
    // Negative coxa rotation swings the right-front leg toward torso +X.
    CLegAngles target_leg_angles_{-20.0, 50.0, 60.0};
    COrientation initial_head_{};
    units::Angle target_head_yaw_ = 0.0 * units::deg;
    units::Angle target_head_pitch_ = -20.0 * units::deg;

    double phase_progress_ = 0.0;
    int hold_iterations_remaining_ = 0;
};

}  // namespace brain
