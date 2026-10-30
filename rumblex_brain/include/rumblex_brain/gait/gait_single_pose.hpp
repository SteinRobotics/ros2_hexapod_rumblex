#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "gait/gait_interfaces.hpp"
#include "gait/gait_parameters.hpp"
#include "gait/pose_model.hpp"
#include "movement_request.hpp"
#include "rumblex_utils/body_pose.hpp"
#include "rumblex_utils/linear_interpolation.hpp"

namespace brain {

using namespace mp_units;
using namespace mp_units::si::unit_symbols;

class CSinglePoseGait : public IContinuousGait {
   public:
    CSinglePoseGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> kinematics,
                    Parameters::SinglePose& params,
                    rumblex_interfaces::msg::BodyPose target = rumblex_interfaces::msg::BodyPose());
    ~CSinglePoseGait() override = default;

    void setTarget(const rumblex_interfaces::msg::BodyPose& target,
                   std::optional<PoseDelta> delta = std::nullopt) {
        options_ = MovementRequest();
        delta_ = std::move(delta);
        target_ = target;
        skip_if_at_target_ = false;
    }
    void setRequest(const MovementRequest& request) {
        setTarget(request.body_pose, request.pose_delta);
        options_ = request;
    }
    bool hasSavedPose() const {
        return saved_pose_.has_value();
    }
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
    rumblex_interfaces::msg::BodyPose target_;
    bool skip_if_at_target_;
    std::optional<PoseDelta> delta_;
    MovementRequest options_;
    std::optional<rumblex_interfaces::msg::BodyPose> saved_pose_;
    std::map<ELegIndex, CPosition> toe_targets_;
    std::map<ELegIndex, CPosition> toe_origins_;
    EGaitState state_ = EGaitState::Stopped;

    CPose torso_target_;
    COrientation head_target_;
    CPose torso_origin_ = CPose();
    COrientation head_origin_ = COrientation();

    units::Duration duration_s_ = 0.0 * s;
    units::Duration phase_ = 0.0 * s;
};

}  // namespace brain
