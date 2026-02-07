#include "phnx_io_ros/pid_interface.hpp"

PidInterface::PidInterface(std::function<void(std::tuple<double, phnx_control::SpeedController::Actuator>)> cb)
    : cb(std::move(cb)) {
    // Setup control thread
    this->thread = std::thread{[this]() {
        // This loop runs at the speed of odom
        while (!this->stop_flag.load()) {
            // Wait for feedback
            nav_msgs::msg::Odometry odom;
            this->odom_queue.wait_dequeue(odom);

            // Always set speed, even if not updated, to avoid queuing latency on commands
            {
                std::unique_lock lk{this->command_mtx};
                this->pid.update_set_speed(this->current_command.speed);
            }

            // Get control
            auto ret = this->pid.update(odom.twist.twist.linear.x, odom.header.stamp);

            // Call callback
            this->cb(ret);
        }
    }};
}

void PidInterface::add_feedback(const nav_msgs::msg::Odometry& speed) { this->odom_queue.enqueue(speed); }

void PidInterface::set_command(const ackermann_msgs::msg::AckermannDrive& command) {
    std::unique_lock lk{this->command_mtx};
    this->current_command = command;
}
