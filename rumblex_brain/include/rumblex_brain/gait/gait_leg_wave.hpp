#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "gait/gait_interfaces.hpp"
#include "gait/gait_parameters.hpp"
#include "gait/pose_model.hpp"
#include "rumblex_utils/geometry.hpp"

namespace brain {

class CLegWaveGait : public ISequenceGait {
   public:
    CLegWaveGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> kinematics,
                 Parameters::LegWave& params);
    ~CLegWaveGait() override = default;

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
    Parameters::LegWave params_;
    EGaitState state_ = EGaitState::Stopped;
    uint8_t direction_ = 0;

    std::map<ELegIndex, CPosition> origins_;
    double phase_ = double(0);
    ELegIndex active_leg_index_ = ELegIndex::RightFront;

    std::vector<ELegIndex> leg_order_ = {ELegIndex::RightFront, ELegIndex::RightMid, ELegIndex::RightBack,
                                         ELegIndex::LeftBack,   ELegIndex::LeftMid,  ELegIndex::LeftFront};
};

}  // namespace brain
