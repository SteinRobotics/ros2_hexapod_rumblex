#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "rumblex_utils/geometry.hpp"
#include "requester/gait_parameters.hpp"
#include "requester/igaits.hpp"
#include "requester/kinematics.hpp"
#include "requester/types.hpp"

namespace rumblex_movement {

class CGaitLegWave : public ISequenceGait {
   public:
    CGaitLegWave(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                 Parameters::LegWave& params);
    ~CGaitLegWave() override = default;

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
    Parameters::LegWave params_;
    EGaitState state_ = EGaitState::Stopped;
    uint8_t direction_ = 0;

    double phase_ = double(0);
    ELegIndex active_leg_index_ = ELegIndex::RightFront;

    std::vector<ELegIndex> leg_order_ = {ELegIndex::RightFront, ELegIndex::RightMid, ELegIndex::RightBack,
                                         ELegIndex::LeftBack,   ELegIndex::LeftMid,  ELegIndex::LeftFront};
};

}  // namespace rumblex_movement
