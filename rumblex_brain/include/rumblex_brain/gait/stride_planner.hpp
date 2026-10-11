#pragma once

#include <vector>

#include "gait/gait_interfaces.hpp"
#include "gait/trajectory.hpp"

namespace brain {

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using mp_units::angular::unit_symbols::deg;

struct StridePattern {
    std::vector<std::vector<ELegIndex>> groups;
    units::Length step = 0.0 * m;
    units::Length lift = 0.0 * m;
    units::Angle head_yaw = 0.0 * deg;
};

// Plans complete contact-to-contact segments. Requests are sampled only at
// touchdown, so changing direction, speed or pattern cannot truncate a swing.
class CStridePlanner {
   public:
    explicit CStridePlanner(std::shared_ptr<CPoseModel> model) : model_(std::move(model)) {
    }
    void start();
    bool update(const StridePattern& pattern, const Velocity& velocity, double rotation_weight,
                const CPose& torso, units::Duration elapsed_s = 0.1 * s);
    void requestStop();
    void cancelStop();
    EGaitState state() const {
        return state_;
    }
    bool atBoundary() const {
        return elapsed_s_ >= duration_s_ - 1e-12 * s;
    }

   private:
    void plan(const StridePattern& pattern, const Velocity& velocity, const CPose& torso);
    std::shared_ptr<CPoseModel> model_;
    EGaitState state_ = EGaitState::Stopped;
    EGaitState resume_state_ = EGaitState::Starting;
    std::map<ELegIndex, CPosition> origins_, targets_;
    std::vector<ELegIndex> swing_;
    // Unwrapped cycle phase carries scheduling progress between patterns.
    bool idle_stop_ = false;
    units::Duration duration_s_ = 0.0 * s;
    units::Duration elapsed_s_ = 0.0 * s;
    units::Duration carry_s_ = 0.0 * s;
    bool settling_segment_ = false;
    Velocity segment_velocity_;
    double cycle_ = 0.0;

    units::Length lift_ = 0.0 * m;
    CPose torso_origin_, torso_target_;
    COrientation head_origin_, head_target_;
};
}  // namespace brain
