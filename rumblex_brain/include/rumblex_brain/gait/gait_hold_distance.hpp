#pragma once

#include "gait/gait_move_combined.hpp"
#include "gait/gait_single_pose.hpp"
#include "gait/hold_distance_control.hpp"

namespace brain {
class CHoldDistanceGait : public ISequenceGait {
   public:
    CHoldDistanceGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> model,
                      Parameters& params);
    void setDistance(units::Length distance) {
        target_distance_ = distance;
    }
    void start(units::Duration duration, uint8_t direction) override;
    bool update() override {
        return updateTimed(0.1 * s);
    }
    bool updateTimed(units::Duration dt) override;
    void requestStop() override;
    void cancelStop() override {
    }  // A new request restarts at rest, with fresh control state.
    EGaitState state() const override {
        return state_;
    }
    std::optional<Velocity> commandedVelocity() const override {
        return velocity_;
    }

   private:
    enum class Phase { Preparing, WaitingSensor, Holding };
    void fail(const std::string& error);
    void finish();
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CPoseModel> model_;
    CSinglePoseGait prepare_;
    CMoveCombinedGait walking_;
    std::unique_ptr<RangeFeedback> sensor_;
    std::unique_ptr<HoldDistanceLaw> control_;
    EGaitState state_ = EGaitState::Stopped;
    Phase phase_ = Phase::Preparing;
    units::Length target_distance_ = 0.0 * m;
    units::Duration duration_ = 0.0 * s;
    units::Duration elapsed_ = 0.0 * s;
    CPose torso_;
    Velocity velocity_;
};
}  // namespace brain
