#include "gait/gait_move_combined.hpp"
namespace brain {
CMoveCombinedGait::CMoveCombinedGait(std::shared_ptr<rclcpp::Node>, std::shared_ptr<CPoseModel> model,
                                     Parameters::Wave& wave, Parameters::Ripple& ripple,
                                     Parameters::Tripod& tripod, Parameters::MoveCombined& combined)
    : planner_(std::move(model)), combined_params_(combined) {
    using enum ELegIndex;
    patterns_[EMoveCombinedGaitType::Wave] = {
        {{RightBack}, {RightMid}, {RightFront}, {LeftBack}, {LeftMid}, {LeftFront}},
        wave.gait_step_length,
        wave.leg_lift_height,
        wave.head_yaw_amplitude};
    patterns_[EMoveCombinedGaitType::Ripple] = {
        {{RightBack, LeftFront}, {RightMid, LeftMid}, {RightFront, LeftBack}},
        ripple.gait_step_length,
        ripple.leg_lift_height,
        ripple.head_yaw_amplitude};
    patterns_[EMoveCombinedGaitType::Tripod] = {
        {{RightFront, LeftMid, RightBack}, {LeftFront, RightMid, LeftBack}},
        tripod.gait_step_length,
        tripod.leg_lift_height,
        tripod.head_yaw_amplitude};
}
EMoveCombinedGaitType CMoveCombinedGait::selectGait(double combined_mag) const {
    const double hyst = combined_params_.hysteresis_margin;
    const double thresh_wr = combined_params_.velocity_threshold_wave_ripple;
    const double thresh_rt = combined_params_.velocity_threshold_ripple_tripod;

    // Large demand changes can cross both thresholds at one contact boundary.
    if (combined_mag >= thresh_rt + hyst) return EMoveCombinedGaitType::Tripod;
    if (combined_mag <= thresh_wr - hyst) return EMoveCombinedGaitType::Wave;
    if (active_gait_type_ == EMoveCombinedGaitType::Wave && combined_mag <= thresh_wr + hyst)
        return EMoveCombinedGaitType::Wave;
    if (active_gait_type_ == EMoveCombinedGaitType::Tripod && combined_mag >= thresh_rt - hyst)
        return EMoveCombinedGaitType::Tripod;
    return EMoveCombinedGaitType::Ripple;
}

bool CMoveCombinedGait::update(const Velocity& velocity, const CPose& torso, const COrientation& head) {
    return updateTimed(velocity, torso, head, 0.1 * units::s);
}

bool CMoveCombinedGait::updateTimed(const Velocity& velocity, const CPose& torso, const COrientation&,
                                    units::Duration elapsed_s) {
    if (planner_.atBoundary() && (state() == EGaitState::Running || state() == EGaitState::Starting)) {
        const auto demand = std::max(
            mp_units::hypot(velocity.linear.x, velocity.linear.y) / combined_params_.max_linear_velocity,
            mp_units::abs(velocity.angular.z) / combined_params_.max_angular_velocity);
        active_gait_type_ = selectGait(std::clamp(demand.numerical_value_in(mp_units::one), 0.0, 1.0));
    }
    return planner_.update(patterns_.at(active_gait_type_), velocity, combined_params_.rotation_weight, torso,
                           elapsed_s);
}
}  // namespace brain
