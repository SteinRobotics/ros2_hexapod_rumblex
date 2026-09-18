#pragma once

#include <cassert>
#include <cmath>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "rumblex_utils/geometry.hpp"
#include "requester/gait_parameters.hpp"
#include "requester/igaits.hpp"
#include "requester/kinematics.hpp"
#include "requester/types.hpp"

namespace rumblex_movement {

class CGaitBodyRoll : public ISequenceGait {
   public:
    CGaitBodyRoll(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                  Parameters::BodyRoll& params);
    ~CGaitBodyRoll() override = default;

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
    EGaitState state_ = EGaitState::Stopped;
    Parameters::BodyRoll params_;
    std::map<ELegIndex, CPosition> origin_leg_positions_;
    double phase_increment_ = 0.1;
    double phase_ = double(0);
};

}  // namespace rumblex_movement
