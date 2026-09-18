#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "rumblex_utils/filters.hpp"
#include "rumblex_utils/linear_interpolation.hpp"
#include "requester/gait_parameters.hpp"
#include "requester/igaits.hpp"
#include "requester/kinematics.hpp"
#include "requester/types.hpp"

namespace rumblex_movement {

class CGaitContinuousPose : public IContinuousGait {
   public:
    CGaitContinuousPose(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                        Parameters::ContinuousPose& params);
    ~CGaitContinuousPose() override = default;

    void start(double duration_s, uint8_t direction) override;
    bool update(const geometry_msgs::msg::Twist& velocity, const CPose& body,
                const COrientation& head) override;
    void requestStop() override;
    void cancelStop() override;
    EGaitState state() const override {
        return state_;
    }

   private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;
    Parameters::ContinuousPose params_;
    EGaitState state_ = EGaitState::Stopped;

    CPose body_origin_ = CPose();
    COrientation head_origin_ = COrientation();
    CPose body_target_ = CPose();
    COrientation head_target_ = COrientation();
};

}  // namespace rumblex_movement
