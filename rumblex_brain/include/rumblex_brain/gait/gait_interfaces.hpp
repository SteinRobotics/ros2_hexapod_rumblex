#pragma once

#include <optional>
#include <utility>

#include "gait/pose_model.hpp"
#include "rclcpp/rclcpp.hpp"
#include "velocity.hpp"

namespace brain {

enum class EGaitState { Starting, Running, StopPending, Stopping, Stopped };

class IGait {
   public:
    virtual ~IGait() = default;

    // start captures the current model without changing it. Stop requests must
    // finish a trajectory at rest before reporting Stopped; cancellation must
    // preserve an in-flight segment. This lets the controller hand off arbitrary
    // behaviors without a pose reset or an output blending layer.
    virtual void start(units::Duration duration_s, uint8_t direction) = 0;
    virtual void requestStop() = 0;
    virtual void cancelStop() = 0;
    virtual EGaitState state() const = 0;
    // Autonomous gaits can report their own velocity.
    virtual std::optional<Velocity> commandedVelocity() const {
        return std::nullopt;
    }
    // Consumed by the planner before another step from the sequence is dispatched.
    bool takeSequenceAbort() {
        return std::exchange(abort_sequence_, false);
    }

   protected:
    bool abort_sequence_ = false;
};

class IContinuousGait : public IGait {
   public:
    virtual bool update(const Velocity& velocity, const CPose& torso, const COrientation& head) = 0;
    virtual bool updateTimed(const Velocity& velocity, const CPose& torso, const COrientation& head,
                             units::Duration /*elapsed_s*/) {
        return update(velocity, torso, head);
    }
};

class ISequenceGait : public IGait {
   public:
    virtual bool update() = 0;
    virtual bool updateTimed(units::Duration /*elapsed_s*/) {
        return update();
    }
};

}  // namespace brain
