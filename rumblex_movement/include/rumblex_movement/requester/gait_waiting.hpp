#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "rumblex_utils/geometry.hpp"
#include "requester/gait_parameters.hpp"
#include "requester/igaits.hpp"
#include "requester/kinematics.hpp"
#include "requester/types.hpp"

namespace rumblex_movement {

// A simple gait that keeps the robot in a waiting (standing) posture while optionally cycling a single leg.
// Initially copied from CLegRollGait and renamed. Behavior can be specialized later.
class CWaitingGait : public ISequenceGait {
   public:
    CWaitingGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                 Parameters::Waiting& params);
    ~CWaitingGait() override = default;

    void start(double duration_s, uint8_t direction) override;
    bool update() override;
    void requestStop() override;
    void cancelStop() override;
    EGaitState state() const override {
        return state_;
    }

   private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;
    Parameters::Waiting params_;

    EGaitState state_ = EGaitState::Stopped;

    double phase_ = double(0);
};

}  // namespace rumblex_movement
