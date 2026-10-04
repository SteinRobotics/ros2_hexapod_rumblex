#pragma once

#include <memory>

#include "gait/gait_interfaces.hpp"
#include "gait/pose_model.hpp"

namespace brain {

// Finite yaw excursions return to the captured pose before handing off.
class CYawSequenceGait : public ISequenceGait {
   public:
    enum class Sweep { OneSide, BothSides };
    CYawSequenceGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> kinematics, Sweep sweep,
                     units::Angle head_amplitude, units::Angle torso_amplitude);

    void start(double duration_s, uint8_t direction) override;
    bool update() override;
    // Stop requests do not interrupt these finite excursions.
    void requestStop() override {
    }
    void cancelStop() override {
    }
    EGaitState state() const override {
        return state_;
    }

   private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CPoseModel> kinematics_;
    double phase_limit_;
    units::Angle head_amplitude_;
    units::Angle torso_amplitude_;
    double direction_sign_ = 1.0;
    double delta_phase_ = 0.0;
    double phase_ = 0.0;
    CPose torso_origin_;
    COrientation head_origin_;
    std::map<ELegIndex, CPosition> toe_origins_;
    EGaitState state_ = EGaitState::Stopped;
};

}  // namespace brain
