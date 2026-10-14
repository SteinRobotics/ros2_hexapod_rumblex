/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include "rclcpp/rclcpp.hpp"

namespace brain {

class IHandler {
   public:
    virtual ~IHandler() = default;

    virtual void update() = 0;
    virtual void cancel() = 0;

    // Finished means this request no longer blocks scheduling. It does not imply
    // success or that an asynchronous operation (including motion) has stopped.
    bool requestFinished() const {
        return request_finished_;
    }

    void setRequestFinished(bool state) {
        request_finished_ = state;
    }

   private:
    bool request_finished_ = true;
};

}  // namespace brain