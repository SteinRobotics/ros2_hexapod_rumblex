#include "gait/stride_planner.hpp"

#include <mp-units/math.h>

namespace brain {
void CStridePlanner::start() {
    state_ = EGaitState::Starting;
    idle_stop_ = false;
    phase_delta_ = 0.4;
    tick_ = ticks_ = 0;
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

void CStridePlanner::plan(const StridePattern& pattern, const geometry_msgs::msg::Twist& velocity,
                          double magnitude, const CPose& torso) {
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
    if (stopping) {
        state_ = EGaitState::Stopping;
        // Settle one group at a time; never drag every foot to an ideal stance.
        for (const auto index : swing_) targets_.at(index) = standing.at(index);
        if (targets_ == origins_) swing_.clear();
    } else {
        state_ = EGaitState::Running;
        for (auto& [index, target] : targets_) {
            const auto& base = standing.at(index);
            const auto radius = mp_units::hypot(base.x, base.y);
            const double tangent_x =
                radius > 1e-6 * units::m ? (-base.y / radius).numerical_value_in(mp_units::one) : 0.0;
            const double tangent_y =
                radius > 1e-6 * units::m ? (base.x / radius).numerical_value_in(mp_units::one) : 0.0;
            const double x = (velocity.linear.x + tangent_x * velocity.angular.z) / magnitude;
            const double y = (velocity.linear.y + tangent_y * velocity.angular.z) / magnitude;
            if (std::ranges::contains(swing_, index)) {
                target = base;
                target.x += pattern.step * x;
                target.y += pattern.step * y;
            } else {
                // Stance travel balances a complete swing across the remaining groups.
                target.x -= pattern.step * (2.0 * x / (count - 1));
                target.y -= pattern.step * (2.0 * y / (count - 1));
                // A change in group count must not accumulate stance travel past
                // the configured workspace before this leg next gets a swing.
                const auto reach = pattern.step * (1.0 + std::abs(velocity.angular.z / magnitude));
                target.x = std::clamp(target.x, base.x - reach, base.x + reach);
                target.y = std::clamp(target.y, base.y - reach, base.y + reach);
            }
        }
        head_target_.yaw = pattern.head_yaw * std::sin(2.0 * M_PI * (cycle_ + 0.5 / count));
    }
    // Phase gains retain their original per-100ms-update units. Freeze duration
    // for this segment, and always sample enough points to resolve touchdown.
    if (!stopping) phase_delta_ = std::max(pattern.phase_gain * magnitude, 0.1);
    const double delta = phase_delta_;
    ticks_ = std::max(10, static_cast<int>(std::ceil(2.0 * M_PI / (count * delta))));
    tick_ = 0;
    cycle_ += 1.0 / count;
}

bool CStridePlanner::update(const StridePattern& pattern, const geometry_msgs::msg::Twist& velocity,
                            double rotation_weight, const CPose& torso) {
    const double magnitude =
        std::sqrt(velocity.linear.x * velocity.linear.x + velocity.linear.y * velocity.linear.y +
                  rotation_weight * velocity.angular.z * velocity.angular.z);
    if (idle_stop_ && magnitude >= 1e-6) {
        // Velocity-driven idle can resume even after settlement. An explicit
        // behavior stop is never cancelled by a stale nonzero velocity input.
        idle_stop_ = false;
        state_ = EGaitState::Running;
    }
    if (state_ == EGaitState::Stopped) return false;
    if (state_ == EGaitState::StopPending && ticks_ == 0) {
        // No segment has started: the current pose is already a rest endpoint.
        state_ = EGaitState::Stopped;
        return false;
    }
    if (atBoundary()) {
        if (magnitude < 1e-6 && state_ == EGaitState::Starting) return false;
        if (magnitude < 1e-6 && state_ == EGaitState::Running) {
            requestStop();
            idle_stop_ = true;
        }
        if ((state_ == EGaitState::Stopping || state_ == EGaitState::StopPending) &&
            model_->getToePositions() == model_->getStandingToePositions() &&
            model_->getTorsoPose() == torso && model_->getHeadOrientation() == COrientation()) {
            state_ = EGaitState::Stopped;
            return false;
        }
        plan(pattern, velocity, magnitude, torso);
    }
    const double t = static_cast<double>(++tick_) / ticks_;
    const double progress = trajectoryProgress(t);
    auto toes = origins_;
    for (auto& [index, toe] : toes) {
        toe = tick_ == ticks_ ? targets_.at(index) : toe.linearInterpolate(targets_.at(index), progress);
        if (std::ranges::contains(swing_, index)) toe.z += lift_ * trajectoryLift(t);
    }
    model_->moveTorso(
        toes, tick_ == ticks_ ? torso_target_ : torso_origin_.linearInterpolate(torso_target_, progress));
    model_->setHeadOrientation(tick_ == ticks_ ? head_target_
                                               : head_origin_.linearInterpolate(head_target_, progress));
    if (tick_ == ticks_ && state_ == EGaitState::Stopping && toes == model_->getStandingToePositions()) {
        state_ = EGaitState::Stopped;
    }
    return true;
}
}  // namespace brain
