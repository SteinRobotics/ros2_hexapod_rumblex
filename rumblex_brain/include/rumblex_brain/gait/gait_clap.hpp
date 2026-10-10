#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "gait/gait_interfaces.hpp"
#include "gait/gait_parameters.hpp"
#include "gait/pose_model.hpp"

namespace brain {

class CClapGait : public ISequenceGait {
   public:
    CClapGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> kinematics,
              Parameters::Clap& params);
    ~CClapGait() override = default;

    void start(units::Duration duration_s, uint8_t direction) override;
    bool update() override;
    void requestStop() override;
    void cancelStop() override;
    EGaitState state() const override {
        return state_;
    }

   private:
    enum class EPhase {
        Idle,
        ShiftingBack,
        LiftRightBack,
        LowerRightBack,
        LiftLeftBack,
        LowerLeftBack,
        LiftFrontLegs,
        ClapClosing,
        ClapOpening,
        LowerFrontLegs,
        ShiftingForward,
        Finished
    };

    void applyTorsoShift(double alpha);
    void applyBackLegLift(ELegIndex leg, double alpha);
    void applyFrontLegsLift(double alpha);
    void applyFrontLegsClap(double alpha);

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CPoseModel> kinematics_;
    Parameters::Clap params_;

    EGaitState state_ = EGaitState::Stopped;
    EPhase phase_ = EPhase::Idle;

    // Store initial positions
    std::map<ELegIndex, CPosition> initial_toe_positions_;
    CPose initial_torso_pose_;
    std::map<ELegIndex, CLegAngles> clap_origin_angles_;

    double phase_progress_ = 0.0;
    int clap_iterations_remaining_ = 0;
};

}  // namespace brain
