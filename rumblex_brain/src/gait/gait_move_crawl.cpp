#include "gait/gait_move_crawl.hpp"
namespace brain {
CMoveCrawlGait::CMoveCrawlGait(std::shared_ptr<rclcpp::Node>, std::shared_ptr<CPoseModel> model,
                               Parameters::Crawl& params)
    : planner_(std::move(model)) {
    using enum ELegIndex;
    // A rear-to-front wave: matching left/right legs swing while four feet support the body.
    pattern_ = {{{RightBack, LeftBack}, {RightMid, LeftMid}, {RightFront, LeftFront}},
                params.gait_step_length,
                params.leg_lift_height,
                params.head_yaw_amplitude};
}
}  // namespace brain
