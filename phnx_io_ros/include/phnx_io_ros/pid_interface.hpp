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
    phnx_control::SpeedController pid;
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

    //Values to limit set speed increase
    double limit = 0.04; //increase from previous set speed in m/s every 1/odom speed(30hz) of a second
    double limSpeed = 0;

public:
    explicit PidInterface(std::function<void(std::tuple<double, phnx_control::SpeedController::Actuator>)> cb, double kP, double kI, double kD); 

    /// Add speed of vehicle to feedback the PID. This runs the control loop, and ultimately calls the callback with
    /// the result.
    void add_feedback(const nav_msgs::msg::Odometry& speed);

//Functions for getting values from the speed_control class:
    //Gets PID Components
    std::tuple<double, double, double, double, double> interface_get_components();
    //Gets PID Coefficients
    std::tuple<double, double, double> interface_get_coeffs();
    //Sets PID Coefficients
    void interface_set_coeffs(double kp, double ki, double kd);
    
    /// Sets the desired speed of the vehicle.
    void set_command(const ackermann_msgs::msg::AckermannDrive& command);

    ~PidInterface() { stop_flag.store(true); }
};
