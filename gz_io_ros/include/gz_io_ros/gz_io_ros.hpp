#pragma once

#include <rclcpp/rclcpp.hpp>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

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

    ///@brief Convert odom/twist data to one Ackeramann message in defined format
    ///@param odom Odom message to convert
    ///@param twist Twist message to convert
    ackermann_msgs::msg::AckermannDrive convert_data(nav_msgs::msg::Odometry::ConstSharedPtr odom,
                                                     geometry_msgs::msg::Twist::ConstSharedPtr twist) const;

    ///@breif Callback for new odom messages
    ///@param odom New message from topic
    void odom_cb(nav_msgs::msg::Odometry::SharedPtr odom);

    ///@breif Callback for new twist messages
    ///@param twist New message from topic
    void twist_cb(geometry_msgs::msg::Twist::SharedPtr twist);

    ///@breif publishes ackermann message
    ///@param msg message to publish
    void publish(ackermann_msgs::msg::AckermannDrive msg);

private:
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr _odom_acks_pub;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _twist_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;

    std::list<geometry_msgs::msg::Twist::SharedPtr> twist_queue;
    std::list<nav_msgs::msg::Odometry::SharedPtr> odom_queue;

    ackermann_msgs::msg::AckermannDrive converted_msg{};

    unsigned long max_buf_size{15};
    double _wheelbase{};
};

}  // namespace gir