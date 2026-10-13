// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace sim::core {

// Simple task-based thread pool used for non-real-time work (asset loading,
// mod scanning, ...). The physics thread is NOT driven by this pool: it has its
// own dedicated thread for deterministic scheduling.
class JobSystem {
public:
    // threadCount == 0 selects std::thread::hardware_concurrency().
    explicit JobSystem(std::uint32_t threadCount = 0);
    ~JobSystem();

    JobSystem(const JobSystem&) = delete;
    JobSystem& operator=(const JobSystem&) = delete;

    void enqueue(std::function<void()> job);
    void waitIdle();

    std::uint32_t threadCount() const { return threadCount_; }
    std::size_t pendingJobs() const { return pending_.load(std::memory_order_acquire); }

private:
    void workerLoop();

    std::vector<std::thread> workers_;
    std::deque<std::function<void()>> queue_;
    mutable std::mutex mutex_;
    std::condition_variable workAvailable_;
    std::condition_variable idle_;
    bool stopping_ = false;
    std::atomic<std::size_t> pending_{0};
    std::uint32_t threadCount_ = 0;
};

} // namespace sim::core
