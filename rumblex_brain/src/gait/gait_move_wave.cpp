#include "gait/gait_move_wave.hpp"
namespace brain {
CMoveWaveGait::CMoveWaveGait(std::shared_ptr<rclcpp::Node>, std::shared_ptr<CPoseModel> model,
                             Parameters::Wave& params)
    : planner_(std::move(model)) {
    using enum ELegIndex;
    pattern_ = {{{RightBack}, {RightMid}, {RightFront}, {LeftBack}, {LeftMid}, {LeftFront}},
                params.gait_step_length,
                params.leg_lift_height,
                params.head_yaw_amplitude};
}
}  // namespace brain
