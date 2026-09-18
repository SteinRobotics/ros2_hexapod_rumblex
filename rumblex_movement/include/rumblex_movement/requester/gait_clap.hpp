#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "requester/gait_parameters.hpp"
#include "requester/igaits.hpp"
#include "requester/kinematics.hpp"
#include "requester/types.hpp"

namespace rumblex_movement {

class CClapGait : public ISequenceGait {
   public:
    CClapGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
              Parameters::Clap& params);
    ~CClapGait() override = default;

    void start(double duration_s, uint8_t direction) override;
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

    void applyBodyShift(double alpha);
    void applyBackLegLift(ELegIndex leg, double alpha);
    void applyFrontLegsLift(double alpha);
    void applyFrontLegsClap(double alpha, bool closing);

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;
    Parameters::Clap params_;

    EGaitState state_ = EGaitState::Stopped;
    EPhase phase_ = EPhase::Idle;

    // Store initial positions
    std::map<ELegIndex, CPosition> initial_foot_positions_;
    CPose initial_body_pose_;

    double phase_progress_ = 0.0;
    int clap_iterations_remaining_ = 0;
};

}  // namespace rumblex_movement
