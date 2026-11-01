#include "gait/gait_move_tripod.hpp"
namespace brain {
CMoveTripodGait::CMoveTripodGait(std::shared_ptr<rclcpp::Node>, std::shared_ptr<CPoseModel> model,
                                 Parameters::Tripod& params)
    : planner_(std::move(model)) {
    using enum ELegIndex;
    pattern_ = {{{RightFront, LeftMid, RightBack}, {LeftFront, RightMid, LeftBack}},
                params.gait_step_length,
                params.leg_lift_height,
                params.head_yaw_amplitude};
}
}  // namespace brain
