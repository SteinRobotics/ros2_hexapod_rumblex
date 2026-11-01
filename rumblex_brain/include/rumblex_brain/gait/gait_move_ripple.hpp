#pragma once
#include "gait/gait_parameters.hpp"
#include "gait/stride_planner.hpp"
namespace brain {
class CMoveRippleGait : public IContinuousGait {
   public:
    CMoveRippleGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> model,
                    Parameters::Ripple& params);
    void start(units::Duration, uint8_t) override {
        planner_.start();
    }
    void requestStop() override {
        planner_.requestStop();
    }
    void cancelStop() override {
        planner_.cancelStop();
    }
    EGaitState state() const override {
        return planner_.state();
    }
    bool update(const Velocity& velocity, const CPose& torso, const COrientation&) override {
        return planner_.update(pattern_, velocity, torso);
    }

    bool updateTimed(const Velocity& velocity, const CPose& torso, const COrientation&,
                     units::Duration elapsed_s) override {
        return planner_.update(pattern_, velocity, torso, elapsed_s);
    }

   private:
    CStridePlanner planner_;
    StridePattern pattern_;
};
}  // namespace brain
