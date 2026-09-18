#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "rumblex_utils/linear_interpolation.hpp"
#include "requester/gait_parameters.hpp"
#include "requester/igaits.hpp"
#include "requester/kinematics.hpp"
#include "requester/types.hpp"

namespace rumblex_movement {

class CGaitSinglePose : public IContinuousGait {
   public:
    CGaitSinglePose(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                    Parameters::SinglePose& params);
    ~CGaitSinglePose() override = default;

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
    Parameters::SinglePose params_;
    EGaitState state_ = EGaitState::Stopped;

    CPose body_origin_ = CPose();
    COrientation head_origin_ = COrientation();

    double duration_s_ = 0.0;
    double phase_increment_ = 0.1;
    double phase_ = 0.0;
};

}  // namespace rumblex_movement
