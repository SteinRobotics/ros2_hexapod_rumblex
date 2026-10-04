#pragma once
#include "gait/gait_parameters.hpp"
#include "gait/stride_planner.hpp"

namespace brain {
enum class EMoveCombinedGaitType { Wave, Ripple, Tripod };
class CMoveCombinedGait : public IContinuousGait {
   public:
    CMoveCombinedGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> model,
                      Parameters::Wave& wave, Parameters::Ripple& ripple, Parameters::Tripod& tripod,
                      Parameters::MoveCombined& combined);
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
    bool update(const geometry_msgs::msg::Twist& velocity, const CPose& torso,
                const COrientation& head) override;

   private:
    EMoveCombinedGaitType selectGait(double magnitude) const;
    CStridePlanner planner_;
    Parameters::MoveCombined combined_params_;
    EMoveCombinedGaitType active_gait_type_ = EMoveCombinedGaitType::Wave;
    std::map<EMoveCombinedGaitType, StridePattern> patterns_;
};
}  // namespace brain
