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
        wave.head_yaw_amplitude,
        wave.velocity_to_phase_gain};
    patterns_[EMoveCombinedGaitType::Ripple] = {
        {{RightBack, LeftFront}, {RightMid, LeftMid}, {RightFront, LeftBack}},
        ripple.gait_step_length,
        ripple.leg_lift_height,
        ripple.head_yaw_amplitude,
        ripple.velocity_to_phase_gain};
    patterns_[EMoveCombinedGaitType::Tripod] = {
        {{RightFront, LeftMid, RightBack}, {LeftFront, RightMid, LeftBack}},
        tripod.gait_step_length,
        tripod.leg_lift_height,
        tripod.head_yaw_amplitude,
        tripod.velocity_to_phase_gain};
}
EMoveCombinedGaitType CMoveCombinedGait::selectGait(double combined_mag) const {
    const double hyst = combined_params_.hysteresis_margin;
    const double thresh_wr = combined_params_.velocity_threshold_wave_ripple;
    const double thresh_rt = combined_params_.velocity_threshold_ripple_tripod;

    switch (active_gait_type_) {
        case EMoveCombinedGaitType::Wave:
            if (combined_mag > thresh_wr + hyst) return EMoveCombinedGaitType::Ripple;
            return EMoveCombinedGaitType::Wave;
        case EMoveCombinedGaitType::Ripple:
            if (combined_mag < thresh_wr - hyst) return EMoveCombinedGaitType::Wave;
            if (combined_mag > thresh_rt + hyst) return EMoveCombinedGaitType::Tripod;
            return EMoveCombinedGaitType::Ripple;
        case EMoveCombinedGaitType::Tripod:
            if (combined_mag < thresh_rt - hyst) return EMoveCombinedGaitType::Ripple;
            return EMoveCombinedGaitType::Tripod;
    }
    return active_gait_type_;
}

bool CMoveCombinedGait::update(const geometry_msgs::msg::Twist& velocity, const CPose& torso,
                               const COrientation&) {
    if (planner_.atBoundary() && (state() == EGaitState::Running || state() == EGaitState::Starting)) {
        const double magnitude =
            std::sqrt(velocity.linear.x * velocity.linear.x + velocity.linear.y * velocity.linear.y +
                      combined_params_.rotation_weight * velocity.angular.z * velocity.angular.z);
        const double maximum =
            std::sqrt(2.0 * std::pow(combined_params_.max_linear_velocity, 2) +
                      combined_params_.rotation_weight * std::pow(combined_params_.max_angular_velocity, 2));
        active_gait_type_ = selectGait(maximum > 1e-6 ? std::clamp(magnitude / maximum, 0.0, 1.0) : 0.0);
    }
    return planner_.update(patterns_.at(active_gait_type_), velocity, combined_params_.rotation_weight,
                           torso);
}
}  // namespace brain
