#include "gait/gait_hold_distance.hpp"

namespace brain {
using namespace mp_units;
using namespace mp_units::si::unit_symbols;

CHoldDistanceGait::CHoldDistanceGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> model,
                                     Parameters& params)
    : node_(node),
      model_(model),
      prepare_(node, model, params.single_pose),
      walking_(node, model, params.wave, params.ripple, params.tripod, params.move_combined, false) {
}

void CHoldDistanceGait::start(units::Duration duration, uint8_t direction) {
    finish();
    abort_sequence_ = false;
    torso_ = model_->getTorsoPose();
    duration_ = duration;
    elapsed_ = 0.0 * s;
    phase_ = Phase::Preparing;
    try {
        if (!isfinite(duration) || duration <= 0.0 * s || !isfinite(target_distance_) ||
            target_distance_ <= 0.0 * m)
            throw std::invalid_argument("HOLD_DISTANCE requires positive duration and distance");
        if (model_->getToePositions() != model_->getStandingToePositions())
            throw std::invalid_argument("HOLD_DISTANCE requires a standing rest pose");
        sensor_ = std::make_unique<RangeFeedback>(node_);
        control_ = std::make_unique<HoldDistanceLaw>(loadHoldDistanceSettings(node_));
        rumblex_interfaces::msg::BodyPose target;
        target.has_head_orientation = true;
        prepare_.setTarget(target);
        prepare_.start(1.0 * s, direction);
        state_ = EGaitState::Starting;
    } catch (const std::exception& error) {
        fail(error.what());
        finish();
    }
}
void CHoldDistanceGait::finish() {
    state_ = EGaitState::Stopped;
    velocity_ = Velocity();
    sensor_.reset();
    control_.reset();
}
void CHoldDistanceGait::requestStop() {
    if (state_ == EGaitState::Stopped) return;
    velocity_ = Velocity();
    state_ = EGaitState::Stopping;
    if (phase_ == Phase::Preparing)
        prepare_.requestStop();
    else if (phase_ == Phase::Holding)
        walking_.requestStop();
    else
        finish();
}
void CHoldDistanceGait::fail(const std::string& error) {
    RCLCPP_WARN(node_->get_logger(), "HOLD_DISTANCE ended sequence: %s", error.c_str());
    abort_sequence_ = true;
    requestStop();
}
bool CHoldDistanceGait::updateTimed(units::Duration dt) {
    if (state_ == EGaitState::Stopped) return false;
    if (!isfinite(dt) || dt <= 0.0 * s || dt > 0.5 * s) {
        if (state_ != EGaitState::Stopping) fail("Invalid distance gait update interval");
        return false;
    }
    if (phase_ == Phase::Preparing) {
        const bool changed = prepare_.updateTimed(Velocity(), torso_, COrientation(), dt);
        if (prepare_.state() == EGaitState::Stopped) {
            if (state_ == EGaitState::Stopping)
                finish();
            else {
                phase_ = Phase::WaitingSensor;
                elapsed_ = 0.0 * s;
            }
        }
        return changed;
    }
    if (phase_ == Phase::Holding && state_ != EGaitState::Stopping && elapsed_ >= duration_) requestStop();
    if (state_ == EGaitState::Stopping) {
        const bool changed = walking_.updateTimed(Velocity(), torso_, COrientation(), dt);
        if (walking_.state() == EGaitState::Stopped) finish();
        return changed;
    }
    const auto range = sensor_->range();
    if (!range) {
        elapsed_ += dt;
        if (phase_ == Phase::Holding || elapsed_ >= sensor_->startup_timeout_s * s)
            fail("Range or forward-beam TF unavailable, invalid, or stale");
        return false;
    }
    if (phase_ == Phase::WaitingSensor) {
        walking_.start(duration_, 0);
        phase_ = Phase::Holding;
        state_ = EGaitState::Running;
        elapsed_ = 0.0 * s;
    }
    try {
        velocity_.linear.x =
            control_->update(*range, target_distance_.numerical_value_in(m), dt.numerical_value_in(s)) * m /
            s;
    } catch (const std::exception& error) {
        fail(error.what());
        return false;
    }
    elapsed_ += dt;
    return walking_.updateTimed(velocity_, torso_, COrientation(), dt);
}
}  // namespace brain
