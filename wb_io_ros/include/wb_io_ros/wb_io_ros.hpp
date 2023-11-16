#pragma once

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
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
    ackermann_msgs::msg::AckermannDrive control;
};
}  // namespace wb_io_ros