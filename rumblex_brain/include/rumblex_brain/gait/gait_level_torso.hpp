#pragma once

#include "gait/gait_interfaces.hpp"
#include "gait/level_torso_control.hpp"

namespace brain {
class CLevelTorsoGait : public ISequenceGait {
   public:
    CLevelTorsoGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> model)
        : node_(std::move(node)), model_(std::move(model)) {
    }
    void start(units::Duration duration, uint8_t direction) override;
    bool update() override {
        return updateTimed(0.1 * s);
    }
    bool updateTimed(units::Duration dt) override;
    void requestStop() override;
    void cancelStop() override {
    }  // Finite sensor gaits restart through the controller.
    EGaitState state() const override {
        return state_;
    }
    std::optional<Velocity> commandedVelocity() const override {
        return Velocity();
    }

   private:
    void fail(const std::string& error);
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CPoseModel> model_;
    std::unique_ptr<ImuFeedback> sensor_;
    std::unique_ptr<LevelTorsoLaw> control_;
    EGaitState state_ = EGaitState::Stopped;
    units::Duration duration_ = 0.0 * s;
    units::Duration elapsed_ = 0.0 * s;
    CPose torso_;
    COrientation head_;
    std::map<ELegIndex, CPosition> toes_;
};
}  // namespace brain
