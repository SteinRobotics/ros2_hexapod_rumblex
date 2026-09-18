#pragma once

#include <geometry_msgs/msg/twist.hpp>

#include "rclcpp/rclcpp.hpp"
#include "requester/types.hpp"

namespace rumblex_movement {

enum class EGaitState { Starting, Running, StopPending, Stopping, Stopped };

class IGait {
   public:
    virtual ~IGait() = default;

    virtual void start(double duration_s, uint8_t direction) = 0;
    virtual void requestStop() = 0;
    virtual void cancelStop() = 0;
    virtual EGaitState state() const = 0;
};

class IContinuousGait : public IGait {
   public:
    virtual bool update(const geometry_msgs::msg::Twist& velocity, const CPose& body,
                        const COrientation& head) = 0;
};

class ISequenceGait : public IGait {
   public:
    virtual bool update() = 0;
};

}  // namespace rumblex_movement
