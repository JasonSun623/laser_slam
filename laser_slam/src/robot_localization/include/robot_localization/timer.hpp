#ifndef ROBOT_LOCALIZATION_TIMER_H
#define ROBOT_LOCALIZATION_TIMER_H

#include <functional>
#include <chrono>
#include <thread>
#include <atomic>
#include <memory>
#include <mutex>
#include <condition_variable>
#include "ros/ros.h"

namespace RobotLocalization
{

class Timer
{
  public:
    Timer() : stopped_(true), try_to_stop_(false)
    {
    }

    Timer(const Timer &t)
    {
        stopped_ = t.stopped_.load();
        try_to_stop_ = t.try_to_stop_.load();
    }
    ~Timer()
    {
        stop();
    }

    void start(int interval, std::function<void()> task)
    {
        // ROS_DEBUG("timer start");
        if (!stopped_)
        {
            ROS_DEBUG("in timer start: stopped false, return");
            return;
        }
        stopped_ = false;
        std::thread([this, interval, task]() {
            while (!try_to_stop_)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(interval));
                task();
            }
            {
                std::lock_guard<std::mutex> locker(mutex_);
                stopped_ = true;
                stopped_cond_.notify_one();
            }
        }).detach();
    }

    void stop()
    {
        // ROS_DEBUG("timer stop");
        if (stopped_)
        {
            ROS_DEBUG("in timer stop: stopped, return");
            return;
        }

        if (try_to_stop_)
        {
            ROS_DEBUG("in timer stop: try to stop, return");
            return;
        }
        try_to_stop_ = true;
        {
            std::unique_lock<std::mutex> locker(mutex_);
            stopped_cond_.wait(locker, [this] { return stopped_ == true; });
            if (stopped_)
            {
                try_to_stop_ = false;
            }
        }
    }

    template <typename callable, class... arguments>
    void syncWait(int after, callable &&f, arguments &&... args)
    {
        std::function<typename std::result_of<callable(arguments...)>::type()> task(std::bind(std::forward<callable>(f), std::forward<arguments>(args)...));
        std::this_thread::sleep_for(std::chrono::milliseconds(after));
        task();
    }

    template <typename callable, class... arguments>
    void asyncWait(int after, callable &&f, arguments &&... args)
    {
        std::function<typename std::result_of<callable(arguments...)>::type()> task(std::bind(std::forward<callable>(f), std::forward<arguments>(args)...));

        std::thread([after, task]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(after));
            task();
        }).detach();
    }

  private:
    std::atomic<bool> stopped_;

    std::atomic<bool> try_to_stop_;

    std::mutex mutex_;

    std::condition_variable stopped_cond_;
};
}

#endif /*ROBOT_LOCALIZATION_TIMER_H*/
