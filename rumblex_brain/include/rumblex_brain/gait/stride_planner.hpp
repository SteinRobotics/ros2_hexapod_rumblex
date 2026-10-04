#pragma once

#include <vector>

#include "gait/gait_interfaces.hpp"
#include "gait/trajectory.hpp"

namespace brain {

struct StridePattern {
    std::vector<std::vector<ELegIndex>> groups;
    units::Length step = 0.0 * units::m;
    units::Length lift = 0.0 * units::m;
    units::Angle head_yaw = 0.0 * units::deg;
    double phase_gain = 1.0;
};

// Plans complete contact-to-contact segments. Requests are sampled only at
// touchdown, so changing direction, speed or pattern cannot truncate a swing.
class CStridePlanner {
   public:
    explicit CStridePlanner(std::shared_ptr<CPoseModel> model) : model_(std::move(model)) {
    }
    void start();
    bool update(const StridePattern& pattern, const geometry_msgs::msg::Twist& velocity,
                double rotation_weight, const CPose& torso);
    void requestStop();
    void cancelStop();
    EGaitState state() const {
        return state_;
    }
    bool atBoundary() const {
        return tick_ == ticks_;
    }

   private:
    void plan(const StridePattern& pattern, const geometry_msgs::msg::Twist& velocity, double magnitude,
              const CPose& torso);
    std::shared_ptr<CPoseModel> model_;
    EGaitState state_ = EGaitState::Stopped;
    EGaitState resume_state_ = EGaitState::Starting;
    std::map<ELegIndex, CPosition> origins_, targets_;
    std::vector<ELegIndex> swing_;
    // Unwrapped cycle phase carries scheduling progress between patterns.
    bool idle_stop_ = false;
    double phase_delta_ = 0.4;
    double cycle_ = 0.0;
    int tick_ = 0, ticks_ = 0;
    units::Length lift_ = 0.0 * units::m;
    CPose torso_origin_, torso_target_;
    COrientation head_origin_, head_target_;
};
}  // namespace brain
