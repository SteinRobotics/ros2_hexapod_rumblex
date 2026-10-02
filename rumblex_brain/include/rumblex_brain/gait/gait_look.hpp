#pragma once

#include <memory>

#include "gait/gait_interfaces.hpp"
#include "gait/gait_parameters.hpp"
#include "gait/pose_model.hpp"
#include "movement_request.hpp"
#include "rclcpp/rclcpp.hpp"

namespace brain {

class CLookGait : public ISequenceGait {
   public:
    CLookGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> kinematics,
              Parameters::Look& params);
    ~CLookGait() override = default;

    void start(double duration_s, uint8_t direction) override;
    bool update() override;
    void requestStop() override;
    void cancelStop() override;
    EGaitState state() const override {
        return state_;
    }

   private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CPoseModel> kinematics_;
    Parameters::Look params_;

    units::Angle amplitude_head_ = 0.0 * units::deg;
    units::Angle amplitude_torso_ = 0.0 * units::deg;
    double delta_phase_ = 0.0;
    double phase_ = 0.0;

    EGaitState state_ = EGaitState::Stopped;
};

}  // namespace brain
