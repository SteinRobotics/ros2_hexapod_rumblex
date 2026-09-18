#pragma once

#include <memory>

#include "rumblex_interfaces/msg/movement_request.hpp"
#include "rclcpp/rclcpp.hpp"
#include "requester/gait_parameters.hpp"
#include "requester/igaits.hpp"
#include "requester/kinematics.hpp"

namespace rumblex_movement {

class CGaitLook : public ISequenceGait {
   public:
    CGaitLook(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
              Parameters::Look& params);
    ~CGaitLook() override = default;

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
    Parameters::Look params_;

    double amplitude_head_deg_ = 0.0;
    double amplitude_body_deg_ = 0.0;
    double delta_phase_ = 0.0;
    double phase_ = 0.0;

    EGaitState state_ = EGaitState::Stopped;
};

}  // namespace rumblex_movement
