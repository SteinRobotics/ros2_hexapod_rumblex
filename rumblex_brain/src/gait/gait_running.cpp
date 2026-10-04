#include "gait/gait_running.hpp"
namespace brain {
CRunningGait::CRunningGait(std::shared_ptr<rclcpp::Node>, std::shared_ptr<CPoseModel> model,
                           Parameters::Running& params)
    : planner_(std::move(model)), rotation_weight_(params.rotation_weight) {
    using enum ELegIndex;
    pattern_ = {{{RightFront, LeftMid, RightBack}, {LeftFront, RightMid, LeftBack}},
                params.gait_step_length,
                params.leg_lift_height,
                params.head_yaw_amplitude,
                params.velocity_to_phase_gain};
}
}  // namespace brain
