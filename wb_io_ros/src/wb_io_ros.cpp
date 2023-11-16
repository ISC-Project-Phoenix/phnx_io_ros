#include "wb_io_ros/wb_io_ros.hpp"

#include <webots/motor.h>
#include <webots/robot.h>

#include <cstdio>
#include <functional>

#include "rclcpp/rclcpp.hpp"

namespace wb_io_ros {
void WbIoRos::init(webots_ros2_driver::WebotsNode* node, std::unordered_map<std::string, std::string>& parameters) {
    ack_sub = node->create_subscription<ackermann_msgs::msg::AckermannDrive>(
        "/ack_vel", rclcpp::SensorDataQoS().reliable(), std::bind(&WbIoRos::ack_cb, this, std::placeholders::_1));

    // TODO get driver api things
}

void WbIoRos::ack_cb(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) { this->control = *msg; }

void WbIoRos::step() {
    // TODO assign controls
}
}  // namespace wb_io_ros

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(wb_io_ros::WbIoRos, webots_ros2_driver::PluginInterface)