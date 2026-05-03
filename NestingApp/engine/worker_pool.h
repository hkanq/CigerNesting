#pragma once

#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>
#include <vector>

namespace nest {

class WorkerPool {
public:
    explicit WorkerPool(size_t threadCount = 0);
    ~WorkerPool();

    WorkerPool(const WorkerPool&) = delete;
    WorkerPool& operator=(const WorkerPool&) = delete;

    template <class Fn>
    auto enqueue(Fn&& fn) -> std::future<decltype(fn())> {
        using Result = decltype(fn());
        auto task = std::make_shared<std::packaged_task<Result()>>(std::forward<Fn>(fn));
        auto future = task->get_future();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            tasks_.push([task]() { (*task)(); });
        }
        cv_.notify_one();
        return future;
    }

    void stop();
    size_t threadCount() const { return workers_.size(); }

private:
    void workerLoop();

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<std::function<void()>> tasks_;
    std::vector<std::thread> workers_;
    bool stopping_ = false;
};

} // namespace nest
