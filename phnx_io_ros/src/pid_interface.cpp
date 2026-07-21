#include "phnx_io_ros/pid_interface.hpp"
#include <rclcpp/rclcpp.hpp>

PidInterface::PidInterface(std::function<void(std::tuple<double, phnx_control::SpeedController::Actuator>)> cb, double kP, double kI, double kD)
    : cb(std::move(cb)) {
    //PID Value params
    
    phnx_control::SpeedController pid(kP, kI, kD);
        
    // Setup control thread
    this->thread = std::thread{[this]() {
        // This loop runs at the speed of odom
        while (!this->stop_flag.load()) {
            // Wait for feedback
            nav_msgs::msg::Odometry odom;
            this->odom_queue.wait_dequeue(odom);

            // Ensure feedback is in valid range, since we want to zero out the encoder 
            // when its below the values we care about. 
            float zero_outter = 0.20;
            if (odom.twist.twist.linear.x > -zero_outter && odom.twist.twist.linear.x < zero_outter) {
                // zero this out please!
                odom.twist.twist.linear.x = 0;
            }

            // Always set speed, even if not updated, to avoid queuing latency on commands
            {
                std::unique_lock lk{this->command_mtx};
                
                //Limit increase in set speed to PID to prevent motor from shorting ):
                if(current_command.speed > limSpeed + limit){
                    limSpeed=limSpeed+limit;
                }
                else{
                    limSpeed = current_command.speed;
                }
                this->pid.update_set_speed(this->limSpeed);
            }

            // Get control
            auto ret = this->pid.update(odom.twist.twist.linear.x, odom.header.stamp);

            // Call callback
            this->cb(ret);
        }
    }};
}


void PidInterface::add_feedback(const nav_msgs::msg::Odometry& speed) { this->odom_queue.enqueue(speed); }

std::tuple<double, double, double, double, double> PidInterface::interface_get_components() {return this->pid.get_components();}

std::tuple<double, double, double> PidInterface::interface_get_coeffs() {return this->pid.get_coeffs();}

void PidInterface::interface_set_coeffs(double kp, double ki, double kd){this->pid.set_coeffs(kp, ki, kd);}

void PidInterface::set_command(const ackermann_msgs::msg::AckermannDrive& command) {
    std::unique_lock lk{this->command_mtx};
    this->current_command = command;
}
