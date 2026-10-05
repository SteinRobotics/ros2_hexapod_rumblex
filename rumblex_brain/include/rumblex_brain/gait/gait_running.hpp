#pragma once
#include "gait/gait_parameters.hpp"
#include "gait/stride_planner.hpp"
namespace brain {
class CRunningGait : public IContinuousGait {
   public:
    CRunningGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> model,
                 Parameters::Running& params);
    void start(double, uint8_t) override {
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
    bool update(const geometry_msgs::msg::Twist& velocity, const CPose& torso, const COrientation&) override {
        return planner_.update(pattern_, velocity, rotation_weight_, torso);
    }

    bool updateTimed(const geometry_msgs::msg::Twist& velocity, const CPose& torso, const COrientation&,
                     double elapsed_s) override {
        return planner_.update(pattern_, velocity, rotation_weight_, torso, elapsed_s);
    }

   private:
    CStridePlanner planner_;
    StridePattern pattern_;
    double rotation_weight_;
};
}  // namespace brain
