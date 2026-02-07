#pragma once
#include <functional>
#include <thread>

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "phnx_control/speed_control.hpp"
#include "phnx_io_ros/vendor/blockingconcurrentqueue.h"
#include "phnx_io_ros/vendor/concurrentqueue.h"

/// Threadsafe wrapper around PID
class PidInterface {
    /// Controller
    phnx_control::SpeedController pid{0.09, 0.4, 0.2};
    /// Control thread
    std::thread thread;
    /// Odom queue
    moodycamel::BlockingConcurrentQueue<nav_msgs::msg::Odometry> odom_queue{};

    /// Most recent command
    ackermann_msgs::msg::AckermannDrive current_command;
    std::mutex command_mtx{};

    /// Called for each output of the PID
    std::function<void(std::tuple<double, phnx_control::SpeedController::Actuator>)> cb;

    std::atomic<bool> stop_flag{false};

public:
    explicit PidInterface(std::function<void(std::tuple<double, phnx_control::SpeedController::Actuator>)> cb);

    /// Add speed of vehicle to feedback the PID. This runs the control loop, and ultimately calls the callback with
    /// the result.
    void add_feedback(const nav_msgs::msg::Odometry& speed);

    /// Sets the desired speed of the vehicle.
    void set_command(const ackermann_msgs::msg::AckermannDrive& command);

    ~PidInterface() { stop_flag.store(true); }
};