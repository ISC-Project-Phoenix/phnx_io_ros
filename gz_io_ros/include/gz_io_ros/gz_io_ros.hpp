#pragma once

#include <rclcpp/rclcpp.hpp>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "optional"

namespace gir {

static const geometry_msgs::msg::Twist zero_twist = [] {
    geometry_msgs::msg::Twist t;
    t.angular.x = 0.0;
    t.angular.y = 0.0;
    t.angular.z = 0.0;
    t.linear.x = 0.0;
    t.linear.y = 0.0;
    t.linear.z = 0.0;
    return t;
}();

class GzIoRos : public rclcpp::Node {
public:
    explicit GzIoRos(rclcpp::NodeOptions options);

private:
    std::optional<std::shared_ptr<rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>>> _odom_acks_pub =
        std::nullopt;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _twist_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;

    std::list<geometry_msgs::msg::Twist::SharedPtr> twist_queue;
    std::list<nav_msgs::msg::Odometry::SharedPtr> odom_queue;

    double _max_throttle_speed{};
    double _max_braking_speed{};
    double _wheelbase{};

    void convert_data(nav_msgs::msg::Odometry::ConstSharedPtr odom, geometry_msgs::msg::Twist::ConstSharedPtr twist);

    void odom_cb(nav_msgs::msg::Odometry::SharedPtr odom);

    void twist_cb(geometry_msgs::msg::Twist::SharedPtr twist);

    double convert_trans_rot_vel_to_steering_angle(double vel, double omega, double wheelbase);
};

}  // namespace gir