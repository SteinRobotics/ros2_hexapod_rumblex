#pragma once

#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>

#include "gait/gait_interfaces.hpp"
#include "gait/gait_parameters.hpp"
#include "gait/pose_model.hpp"
#include "rumblex_utils/linear_interpolation.hpp"

namespace brain {

class CSinglePoseGait : public IContinuousGait {
   public:
    struct Target {
        std::map<ELegIndex, CPosition> toes;
        CPose torso;
        COrientation head;
    };

    CSinglePoseGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> kinematics,
                    Parameters::SinglePose& params, std::optional<Target> target = std::nullopt);
    ~CSinglePoseGait() override = default;

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
    std::optional<Target> target_;
    std::map<ELegIndex, CPosition> toe_origins_;
    EGaitState state_ = EGaitState::Stopped;

    CPose torso_target_;
    COrientation head_target_;
    CPose torso_origin_ = CPose();
    COrientation head_origin_ = COrientation();

    units::Duration duration_s_ = 0.0 * units::s;
    units::Duration phase_ = 0.0 * units::s;
};

}  // namespace brain
