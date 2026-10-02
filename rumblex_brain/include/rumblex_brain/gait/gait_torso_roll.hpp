#pragma once

#include <cassert>
#include <cmath>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "gait/gait_interfaces.hpp"
#include "gait/gait_parameters.hpp"
#include "gait/pose_model.hpp"
#include "rumblex_utils/geometry.hpp"

namespace brain {

class CTorsoRollGait : public ISequenceGait {
   public:
    CTorsoRollGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> kinematics,
                   Parameters::TorsoRoll& params);
    ~CTorsoRollGait() override = default;

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
    EGaitState state_ = EGaitState::Stopped;
    EGaitState resume_state_ = EGaitState::Starting;
    Parameters::TorsoRoll params_;
    std::map<ELegIndex, CPosition> origin_leg_positions_;
    double phase_increment_ = 0.1;
    double phase_ = double(0);
};

}  // namespace brain
