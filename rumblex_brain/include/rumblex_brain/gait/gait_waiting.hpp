#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "gait/gait_interfaces.hpp"
#include "gait/gait_parameters.hpp"
#include "gait/pose_model.hpp"
#include "rumblex_utils/geometry.hpp"

namespace brain {

// A simple gait that keeps the robot in a waiting (standing) posture while optionally cycling a single leg.
// Initially copied from CLegRollGait and renamed. Behavior can be specialized later.
class CWaitingGait : public ISequenceGait {
   public:
    CWaitingGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> kinematics,
                 Parameters::Waiting& params);
    ~CWaitingGait() override = default;

    void start(units::Duration duration_s, uint8_t direction) override;
    bool update() override;
    void requestStop() override;
    void cancelStop() override;
    EGaitState state() const override {
        return state_;
    }

   private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CPoseModel> kinematics_;
    Parameters::Waiting params_;

    EGaitState state_ = EGaitState::Stopped;

    CPose torso_origin_;
    std::map<ELegIndex, CPosition> toe_origins_;
    units::Angle phase_ = 0.0 * units::rad;
};

}  // namespace brain
