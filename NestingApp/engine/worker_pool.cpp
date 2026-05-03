#include "engine/worker_pool.h"

#include <algorithm>

namespace nest {

WorkerPool::WorkerPool(size_t threadCount) {
    if (threadCount == 0) {
        threadCount = std::max<size_t>(1, std::thread::hardware_concurrency());
    }
    workers_.reserve(threadCount);
    for (size_t i = 0; i < threadCount; ++i) {
        workers_.emplace_back([this]() { workerLoop(); });
    }
}

WorkerPool::~WorkerPool() {
    stop();
}

void WorkerPool::stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
    }
    cv_.notify_all();
    for (auto& worker : workers_) {
        if (worker.joinable()) {
            worker.join();
        }
    }
    workers_.clear();
}

void WorkerPool::workerLoop() {
    for (;;) {
        std::function<void()> task;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this]() { return stopping_ || !tasks_.empty(); });
            if (stopping_ && tasks_.empty()) {
                return;
            }
            task = std::move(tasks_.front());
            tasks_.pop();
        }
        task();
    }
}

} // namespace nest
