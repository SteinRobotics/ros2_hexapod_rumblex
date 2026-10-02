#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "gait/gait_interfaces.hpp"
#include "gait/gait_parameters.hpp"
#include "gait/pose_model.hpp"
#include "rumblex_utils/linear_interpolation.hpp"

namespace brain {

class CLayDownGait : public ISequenceGait {
   public:
    CLayDownGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> kinematics,
                 Parameters::LayDown& params);
    ~CLayDownGait() override = default;

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
    Parameters::LayDown params_;
    EGaitState state_ = EGaitState::Stopped;
    std::map<ELegIndex, CPosition> target_leg_positions_;
    COrientation target_head_position_;
    std::map<ELegIndex, CPosition> origin_leg_positions_;
    COrientation origin_head_orientation_;
    double phase_increment_ = 0.1;
    double phase_ = 0.0;
};

}  // namespace brain
