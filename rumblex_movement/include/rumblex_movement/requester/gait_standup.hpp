#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "rumblex_utils/linear_interpolation.hpp"
#include "requester/gait_parameters.hpp"
#include "requester/igaits.hpp"
#include "requester/kinematics.hpp"
#include "requester/types.hpp"

namespace rumblex_movement {

class CStandUpGait : public ISequenceGait {
   public:
    CStandUpGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                 Parameters::StandUp& params);
    ~CStandUpGait() override = default;

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
    Parameters::StandUp params_;
    EGaitState state_ = EGaitState::Stopped;
    std::map<ELegIndex, CPosition> target_leg_positions_;
    std::map<ELegIndex, CPosition> origin_leg_positions_;
    COrientation target_head_position_;
    COrientation origin_head_position_;

    double phase_ = 0.0;
    double phase_increment_ = 0.1;
};

}  // namespace rumblex_movement
