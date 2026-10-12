// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "core/JobSystem.hpp"

#include <algorithm>

namespace sim::core {

JobSystem::JobSystem(std::uint32_t threadCount)
{
    if (threadCount == 0)
    {
        threadCount = std::max(1u, std::thread::hardware_concurrency());
    }
    threadCount_ = threadCount;

    workers_.reserve(threadCount_);
    for (std::uint32_t i = 0; i < threadCount_; ++i)
    {
        workers_.emplace_back([this] { workerLoop(); });
    }
}

JobSystem::~JobSystem()
{
    {
        std::lock_guard lock(mutex_);
        stopping_ = true;
    }
    workAvailable_.notify_all();
    for (auto& worker : workers_)
    {
        if (worker.joinable())
        {
            worker.join();
        }
    }
}

void JobSystem::enqueue(std::function<void()> job)
{
    pending_.fetch_add(1, std::memory_order_acq_rel);
    {
        std::lock_guard lock(mutex_);
        queue_.push_back(std::move(job));
    }
    workAvailable_.notify_one();
}

void JobSystem::waitIdle()
{
    std::unique_lock lock(mutex_);
    idle_.wait(lock, [this] { return pending_.load(std::memory_order_acquire) == 0; });
}

void JobSystem::workerLoop()
{
    for (;;)
    {
        std::function<void()> job;
        {
            std::unique_lock lock(mutex_);
            workAvailable_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
            if (stopping_ && queue_.empty())
            {
                return;
            }
            job = std::move(queue_.front());
            queue_.pop_front();
        }

        job();

        if (pending_.fetch_sub(1, std::memory_order_acq_rel) == 1)
        {
            std::lock_guard lock(mutex_);
            idle_.notify_all();
        }
    }
}

} // namespace sim::core
