#include "gait/stride_planner.hpp"

#include <mp-units/math.h>

namespace brain {

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using mp_units::angular::unit_symbols::rad;
namespace {
// Apply exp(t * planar_twist). Negative time keeps a stance foot fixed in the world.
CPosition transformToe(const CPosition& origin, const Velocity& velocity, units::Duration t) {
    const auto angle = velocity.angular.z * t;
    const double c = angular::cos(angle).numerical_value_in(one);
    const double sine = angular::sin(angle).numerical_value_in(one);
    // Distinct angles require an explicit radian when converting rotation to arc motion.
    const bool small_angle = abs(angle) < 1e-8 * rad;
    const units::Duration a = small_angle ? t : sine * rad / velocity.angular.z;
    const units::Duration b = small_angle ? 0.5 * (angle / rad) * t : (1.0 - c) * rad / velocity.angular.z;
    CPosition target = origin;
    target.x = c * origin.x - sine * origin.y + a * velocity.linear.x - b * velocity.linear.y;
    target.y = sine * origin.x + c * origin.y + b * velocity.linear.x + a * velocity.linear.y;
    return target;
}
}  // namespace

void CStridePlanner::start() {
    state_ = EGaitState::Starting;
    idle_stop_ = false;
    duration_s_ = elapsed_s_ = carry_s_ = 0.0 * s;
    cycle_ = 0.0;
}

void CStridePlanner::requestStop() {
    idle_stop_ = false;
    if (state_ == EGaitState::Starting || state_ == EGaitState::Running) {
        resume_state_ = state_;
        state_ = EGaitState::StopPending;
    }
}

void CStridePlanner::cancelStop() {
    idle_stop_ = false;
    if (state_ == EGaitState::StopPending || state_ == EGaitState::Stopping) {
        state_ = resume_state_;
    }
}

void CStridePlanner::plan(const StridePattern& pattern, const Velocity& velocity, const CPose& torso) {
    origins_ = model_->getToePositions();
    targets_ = origins_;
    torso_origin_ = model_->getTorsoPose();
    torso_target_ = torso;
    head_origin_ = model_->getHeadOrientation();
    head_target_ = COrientation();
    const auto standing = model_->getStandingToePositions();
    const auto count = pattern.groups.size();
    const auto group = static_cast<size_t>(std::floor(cycle_ * count + 1e-9)) % count;
    swing_ = pattern.groups.at(group);
    lift_ = pattern.lift;
    const bool stopping = state_ == EGaitState::StopPending || state_ == EGaitState::Stopping;
    settling_segment_ = stopping;
    if (!stopping) {
        units::LinearVelocity foot_speed = 0.0 * m / s;
        for (const auto& [index, toe] : standing) {
            const auto rotation_rate = velocity.angular.z / rad;
            foot_speed = std::max(foot_speed, hypot(velocity.linear.x - rotation_rate * toe.y,
                                                    velocity.linear.y + rotation_rate * toe.x));
        }
        // Shorten the stride at low speeds rather than waiting many seconds for touchdown.
        duration_s_ = std::clamp(2.0 * pattern.step / ((count - 1) * foot_speed), 0.2 * s, 1.0 * s);
        constexpr auto tick = 0.02 * s;
        duration_s_ = std::ceil((duration_s_ / tick).numerical_value_in(one) - 1e-12) * tick;
        segment_velocity_ = velocity;
    } else if (duration_s_ <= 0.0 * s) {
        duration_s_ = 0.2 * s;
    }
    if (stopping) {
        state_ = EGaitState::Stopping;
        // Settle one group at a time; never drag every foot to an ideal stance.
        for (const auto index : swing_) targets_.at(index) = standing.at(index);
        if (targets_ == origins_) swing_.clear();
    } else {
        state_ = EGaitState::Running;
        for (auto& [index, target] : targets_) {
            const auto& base = standing.at(index);
            if (std::ranges::contains(swing_, index)) {
                target = transformToe(base, velocity, duration_s_ * (count - 1) / 2.0);
            } else {
                target = transformToe(origins_.at(index), velocity, -duration_s_);
                // During pattern changes, an old stance may outlast its new schedule.
                // Keep that transient inside the configured workspace until its swing.
                target.x = std::clamp(target.x, base.x - pattern.step, base.x + pattern.step);
                target.y = std::clamp(target.y, base.y - pattern.step, base.y + pattern.step);
            }
        }
        head_target_.yaw = pattern.head_yaw * angular::sin(2.0 * M_PI * rad * (cycle_ + 0.5 / count));
    }
    elapsed_s_ = 0.0 * s;
    cycle_ += 1.0 / count;
}

bool CStridePlanner::update(const StridePattern& pattern, const Velocity& velocity,
                            double /*rotation_weight*/, const CPose& torso, units::Duration elapsed_s) {
    if (!isfinite(elapsed_s) || elapsed_s <= 0.0 * s || elapsed_s > 0.5 * s) {
        carry_s_ = 0.0 * s;
        return false;
    }
    const bool idle = hypot(velocity.linear.x, velocity.linear.y) < 1e-6 * m / s &&
                      abs(velocity.angular.z) < 1e-6 * rad / s;
    if (idle_stop_ && !idle) {
        // Velocity-driven idle can resume even after settlement. An explicit
        // behavior stop is never cancelled by a stale nonzero velocity input.
        idle_stop_ = false;
        state_ = EGaitState::Running;
    }
    if (state_ == EGaitState::Stopped) return false;
    if (state_ == EGaitState::StopPending && duration_s_ == 0.0 * s) {
        // No segment has started: the current pose is already a rest endpoint.
        state_ = EGaitState::Stopped;
        return false;
    }
    if (atBoundary()) {
        if (idle && state_ == EGaitState::Starting) return false;
        if (idle && state_ == EGaitState::Running) {
            requestStop();
            idle_stop_ = true;
        }
        if ((state_ == EGaitState::Stopping || state_ == EGaitState::StopPending) &&
            model_->getToePositions() == model_->getStandingToePositions() &&
            model_->getTorsoPose() == torso && model_->getHeadOrientation() == COrientation()) {
            state_ = EGaitState::Stopped;
            return false;
        }
        plan(pattern, velocity, torso);
    }
    const auto requested_elapsed = elapsed_s_ + elapsed_s + carry_s_;
    elapsed_s_ = std::min(requested_elapsed, duration_s_);
    carry_s_ = std::max(0.0 * s, requested_elapsed - duration_s_);
    const double t = atBoundary() ? 1.0 : (elapsed_s_ / duration_s_).numerical_value_in(one);
    const double progress = trajectoryProgress(t);
    auto toes = origins_;
    for (auto& [index, toe] : toes) {
        const bool swing = std::ranges::contains(swing_, index);
        const auto endpoint = transformToe(origins_.at(index), segment_velocity_, -duration_s_);
        if (!settling_segment_ && !swing && endpoint == targets_.at(index)) {
            toe = transformToe(origins_.at(index), segment_velocity_, -duration_s_ * progress);
        } else {
            toe = toe.linearInterpolate(targets_.at(index), progress);
        }
        if (atBoundary()) toe = targets_.at(index);
        if (std::ranges::contains(swing_, index)) toe.z += lift_ * trajectoryLift(t);
    }
    model_->moveTorso(
        toes, atBoundary() ? torso_target_ : torso_origin_.linearInterpolate(torso_target_, progress));
    model_->setHeadOrientation(atBoundary() ? head_target_
                                            : head_origin_.linearInterpolate(head_target_, progress));
    if (atBoundary() && state_ == EGaitState::Stopping && toes == model_->getStandingToePositions()) {
        state_ = EGaitState::Stopped;
    }
    return true;
}
}  // namespace brain
