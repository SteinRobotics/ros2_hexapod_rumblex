#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "gait/gait_interfaces.hpp"
#include "gait/gait_parameters.hpp"
#include "gait/pose_model.hpp"
#include "rumblex_utils/linear_interpolation.hpp"

namespace brain {

class CSinglePoseGait : public IContinuousGait {
   public:
    CSinglePoseGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> kinematics,
                    Parameters::SinglePose& params);
    ~CSinglePoseGait() override = default;

    void start(double duration_s, uint8_t direction) override;
    bool update(const geometry_msgs::msg::Twist& velocity, const CPose& torso,
                const COrientation& head) override;
    void requestStop() override;
    void cancelStop() override;
    EGaitState state() const override {
        return state_;
    }

   private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CPoseModel> kinematics_;
    Parameters::SinglePose params_;
    EGaitState state_ = EGaitState::Stopped;

    CPose torso_origin_ = CPose();
    COrientation head_origin_ = COrientation();

    double duration_s_ = 0.0;
    double phase_increment_ = 0.1;
    double phase_ = 0.0;
};

}  // namespace brain
