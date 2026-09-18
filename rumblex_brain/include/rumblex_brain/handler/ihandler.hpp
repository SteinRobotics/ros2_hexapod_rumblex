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

    bool done() const {
        return is_done_;
    }

    void setDone(bool state) {
        is_done_ = state;
    }

   private:
    bool is_done_ = true;
};

}  // namespace brain