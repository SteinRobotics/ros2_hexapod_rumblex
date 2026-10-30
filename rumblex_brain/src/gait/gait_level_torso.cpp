#include "gait/gait_level_torso.hpp"

namespace brain {
using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using mp_units::angular::unit_symbols::rad;

void CLevelTorsoGait::start(units::Duration duration, uint8_t) {
    requestStop();
    abort_sequence_ = false;
    duration_ = duration;
    elapsed_ = 0.0 * s;
    torso_ = model_->getTorsoPose();
    head_ = model_->getHeadOrientation();
    toes_ = model_->getToePositions();
    try {
        if (!isfinite(duration) || duration <= 0.0 * s)
            throw std::invalid_argument("LEVEL_TORSO requires positive duration");
        if (toes_ != model_->getStandingToePositions())
            throw std::invalid_argument("LEVEL_TORSO requires a standing rest pose");
        sensor_ = std::make_unique<ImuFeedback>(node_);
        control_ = std::make_unique<LevelTorsoLaw>(loadLevelTorsoSettings(node_));
        control_->reset({torso_.orientation.roll.numerical_value_in(rad),
                         torso_.orientation.pitch.numerical_value_in(rad)});
        state_ = EGaitState::Starting;
    } catch (const std::exception& error) {
        fail(error.what());
    }
}
void CLevelTorsoGait::requestStop() {
    state_ = EGaitState::Stopped;
    sensor_.reset();
    control_.reset();
}
void CLevelTorsoGait::fail(const std::string& error) {
    RCLCPP_WARN(node_->get_logger(), "LEVEL_TORSO ended sequence: %s", error.c_str());
    abort_sequence_ = true;
    requestStop();
}
bool CLevelTorsoGait::updateTimed(units::Duration dt) {
    if (state_ == EGaitState::Stopped) return false;
    if (!isfinite(dt) || dt <= 0.0 * s || dt > 0.5 * s) {
        fail("Invalid leveling gait update interval");
        return false;
    }
    if (state_ == EGaitState::Running && elapsed_ >= duration_) {
        requestStop();
        return false;
    }
    const auto measured = sensor_->tilt();
    if (!measured) {
        elapsed_ += dt;
        if (state_ == EGaitState::Running || elapsed_ >= sensor_->startup_timeout_s * s)
            fail("IMU orientation or mounting TF unavailable, invalid, or stale");
        return false;
    }
    if (state_ == EGaitState::Starting) {
        state_ = EGaitState::Running;
        elapsed_ = 0.0 * s;
    }
    try {
        const auto command = control_->update(*measured, dt.numerical_value_in(s));
        torso_.orientation.roll = command.roll * rad;
        torso_.orientation.pitch = command.pitch * rad;
        model_->moveTorso(toes_, torso_);
        model_->setHeadOrientation(head_);
        elapsed_ += dt;
        return true;
    } catch (const std::exception& error) {
        fail(error.what());
        return false;
    }
}
}  // namespace brain
