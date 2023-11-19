#include "wb_io_ros/wb_io_ros.hpp"

#include <functional>

namespace wb_io_ros {
void WbIoRos::init(webots_ros2_driver::WebotsNode* node, std::unordered_map<std::string, std::string>& parameters) {
    wbu_driver_init();

    // Store owning node
    this->parent = node;

    ack_sub = node->create_subscription<ackermann_msgs::msg::AckermannDrive>(
        "/robot/ack_vel", 5, std::bind(&WbIoRos::ack_cb, this, std::placeholders::_1));
    odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("/odom_can", 10);
}

void WbIoRos::ack_cb(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) { this->control = *msg; }

void WbIoRos::step() {
    wbu_driver_step();

    // Capture encoder value as odom
    nav_msgs::msg::Odometry odom{};
    odom.header.stamp = parent->get_clock()->now();
    // Convert from kph to mps
    odom.twist.twist.linear.x = wbu_driver_get_current_speed() / 7.2;

    // Sometimes we get nans, remove them here else it breaks the ekf
    if (!std::isnan(odom.twist.twist.linear.x)) {
        this->odom_pub->publish(odom);
    } else {
        RCLCPP_INFO(parent->get_logger(), "Skipping nan encoder value");
    }

    // Actuate the car, converting from ros units
    wbu_driver_set_cruising_speed(this->control.speed * 3.6);
    wbu_driver_set_steering_angle(-this->control.steering_angle);
}
}  // namespace wb_io_ros

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(wb_io_ros::WbIoRos, webots_ros2_driver::PluginInterface)