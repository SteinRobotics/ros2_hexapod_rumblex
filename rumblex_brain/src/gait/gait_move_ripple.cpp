#include "gait/gait_move_ripple.hpp"
namespace brain {
CMoveRippleGait::CMoveRippleGait(std::shared_ptr<rclcpp::Node>, std::shared_ptr<CPoseModel> model,
                                 Parameters::Ripple& params)
    : planner_(std::move(model)) {
    using enum ELegIndex;
    pattern_ = {{{RightBack, LeftFront}, {RightMid, LeftMid}, {RightFront, LeftBack}},
                params.gait_step_length,
                params.leg_lift_height,
                params.head_yaw_amplitude};
}
}  // namespace brain
