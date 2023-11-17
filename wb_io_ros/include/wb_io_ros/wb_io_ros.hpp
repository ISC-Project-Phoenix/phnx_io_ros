#pragma once

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "webots/vehicle/driver.h"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

namespace wb_io_ros {
class WbIoRos : public webots_ros2_driver::PluginInterface {
public:
    void step() override;
    void init(webots_ros2_driver::WebotsNode* node, std::unordered_map<std::string, std::string>& parameters) override;

private:
    void ack_cb(ackermann_msgs::msg::AckermannDrive::SharedPtr msg);

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr ack_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    ackermann_msgs::msg::AckermannDrive control;
    webots_ros2_driver::WebotsNode* parent;

    ~WbIoRos() { wbu_driver_cleanup(); }
};
}  // namespace wb_io_ros