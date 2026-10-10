#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "gait/gait_interfaces.hpp"
#include "gait/gait_parameters.hpp"
#include "gait/pose_model.hpp"
#include "rumblex_utils/linear_interpolation.hpp"

namespace brain {

class CContinuousPoseGait : public IContinuousGait {
   public:
    CContinuousPoseGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> kinematics,
                        Parameters::ContinuousPose& params);
    ~CContinuousPoseGait() override = default;

    void start(units::Duration duration_s, uint8_t direction) override;
    bool update(const Velocity& velocity, const CPose& torso, const COrientation& head) override;
    bool updateTimed(const Velocity& velocity, const CPose& torso, const COrientation& head,
                     units::Duration elapsed_s) override;
    void requestStop() override;
    void cancelStop() override;
    EGaitState state() const override {
        return state_;
    }

   private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CPoseModel> kinematics_;
    Parameters::ContinuousPose params_;
    EGaitState state_ = EGaitState::Stopped;

    units::Duration elapsed_s_ = 1.0 * units::s;
    std::map<ELegIndex, CPosition> toe_origins_;
    CPose torso_origin_ = CPose();
    COrientation head_origin_ = COrientation();
    CPose torso_target_ = CPose();
    COrientation head_target_ = COrientation();
};

}  // namespace brain
