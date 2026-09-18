/*******************************************************************************
 * Copyright (c) 2025 Christian Stein
 ******************************************************************************/

#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <thread>

/**
 * CSimpleTimer - small helper for simple timing needs.
 *
 * Notes about current semantics:
 * - getSecondsElapsed() / haveSecondsElapsed() use steady_clock and report
 *   fractional seconds.
 * - start()/stop()/isRunning() manage a simple atomic running flag and the
 *   stored start time.
 * - waitSecondsBlocking(double) performs a blocking sleep on the calling thread
 *   and is safe for short waits used in tests or simple sequencing.
 *
 * Non-blocking behavior (waitSecondsNonBlocking):
 * - Currently this function starts the timer (records start time) and spawns a
 *   detached std::thread which sleeps and then invokes the provided callback.
 * - After the callback executes the implementation calls stop() to clear the
 *   running flag.
 * - Important: stop() only clears the running flag. It does NOT cancel the
 *   sleeping detached thread in the current implementation. That means a
 *   scheduled non-blocking callback will still run even if stop() is called
 *   before the sleep expires.
 */

namespace utils {

class CSimpleTimer {
   public:
    explicit CSimpleTimer(bool automaticStart = false) {
        if (automaticStart) {
            start();
        }
    }

    ~CSimpleTimer() {
        // Note: stop() only clears the running flag in this implementation.
        // It does not cancel any detached thread spawned by
        // waitSecondsNonBlocking(). See the class notes above for migration
        // suggestions if you need cancellation on destruction.
        stop();
    }

    /// Start the timer (records start time and marks running).
    void start() {
        start_time_ = std::chrono::steady_clock::now();
        is_running_ = true;
    }

    /// Stop the timer (clears running flag). Does not cancel detached threads.
    void stop() {
        is_running_ = false;
    }

    bool isRunning() const {
        return is_running_;
    }

    /// Seconds elapsed since start() as fractional seconds. Returns 0 if not
    /// running.
    double getSecondsElapsed() const {
        if (!is_running_) return 0;
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - start_time_;
        return elapsed.count();
    }

    bool haveSecondsElapsed(double seconds) const {
        return getSecondsElapsed() >= seconds;
    }

    /**
     * Schedule callback to be executed after `seconds` in a new detached
     * thread. The timer is started immediately. After the callback executes
     * the timer's running flag will be cleared.
     *
     * Warning: the spawned thread is detached and cannot be cancelled by
     * stop(). If you need cancellation, consider migrating this method to
     * use std::jthread (C++20) or use CCallbackTimer which already uses
     * std::jthread for cooperative cancellation.
     */
    // void waitSecondsNonBlocking(double seconds, std::function<void()> callback) {
    //     start();
    //     // Run the callback in a detached thread and stop the timer after callback runs.
    //     std::thread([seconds, callback, this]() {
    //         std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
    //         if (isRunning_) {
    //             callback();
    //         }
    //         // mark timer stopped after callback
    //         this->stop();
    //     }).detach();
    // }

   private:
    std::chrono::steady_clock::time_point start_time_;
    std::atomic<bool> is_running_ = false;
};

}  // namespace utils

// Backward-compat: keep un-namespaced alias for now
using CSimpleTimer = utils::CSimpleTimer;
