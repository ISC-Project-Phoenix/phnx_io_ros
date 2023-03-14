#include <functional>
#include <rclcpp/rclcpp.hpp>

#include "gz_io_ros/gz_io_ros.hpp"
#include "libackermann/libackermann.hpp"

gir::GzIoRos::GzIoRos(rclcpp::NodeOptions options) : Node("gz_io_ros", options) {
    rclcpp::QoS qos(50);
    _max_throttle_speed = this->declare_parameter("max_throttle_speed", 10.0);
    _max_braking_speed = this->declare_parameter("max_brake_speed", -10.0);
    _max_steering_rad = this->declare_parameter("max_steering_rad", 2.0);
    _wheelbase = this->declare_parameter("wheelbase", 1.0);

    _odom_acks_pub = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("/odom_ack", 10);

    _twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "/robot/cmd_vel", rclcpp::QoS(10).reliable(), std::bind(&GzIoRos::twist_cb, this, std::placeholders::_1));

    _odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::QoS(10).reliable(), std::bind(&GzIoRos::odom_cb, this, std::placeholders::_1));
}

void gir::GzIoRos::convert_data(nav_msgs::msg::Odometry::ConstSharedPtr odom,
                                geometry_msgs::msg::Twist::ConstSharedPtr twist) {
    //Convert odom + twist msg to ackermann drive msg

    TwistCommand t{static_cast<float>(twist->linear.x), static_cast<float>(twist->angular.z)};
    AckermannCommand a = ack::twist_to_ackermann(t, this->_wheelbase);

    auto ratio = ack::get_inverse_steering_ratio(ack::Project::Phoenix);

    // ack odom messages carry steering wheel in steering field, rather than ackermann wheel angle
    converted_msg.steering_angle = ratio(a.ackermann_angle);

    // Throttle percent (encoded as a speed as per a twist message) is in the accell field
    converted_msg.acceleration = twist->linear.x;

    // Encoder values (odom in sims case) is in the speed field
    converted_msg.speed = odom->twist.twist.linear.x;
    converted_msg.jerk = 0.0;
    validate_msg(converted_msg);
    this->_odom_acks_pub->publish(converted_msg);
}

void gir::GzIoRos::odom_cb(nav_msgs::msg::Odometry::SharedPtr odom) {
    //Since twist messages aren't constant there will be situations where we have odom and no twist,
    // in that case send zero twist to make sure those values are zeroed out
    if (odom_queue.size() > max_buf_size) {
        odom_queue.clear();
    }
    this->odom_queue.push_back(odom);

    if (!this->odom_queue.empty()) {
        if (this->twist_queue.empty()) {
            convert_data(odom_queue.front(), std::make_shared<geometry_msgs::msg::Twist>(zero_twist));
        } else {
            convert_data(odom_queue.front(), twist_queue.front());
            twist_queue.pop_front();
        }
        odom_queue.pop_front();
    }
}

void gir::GzIoRos::twist_cb(geometry_msgs::msg::Twist::SharedPtr twist) {
    //Since odom messages are constant as long as sim is running its fine to ensure both queues have data
    if (twist_queue.size() > max_buf_size) {
        twist_queue.clear();
    }
    this->twist_queue.push_back(twist);

    if (!this->twist_queue.empty() && !this->odom_queue.empty()) {
        convert_data(odom_queue.front(), twist_queue.front());
        odom_queue.pop_front();
        twist_queue.pop_front();
    }
}
void gir::GzIoRos::validate_msg(ackermann_msgs::msg::AckermannDrive& msg) {
    if (msg.acceleration < _max_braking_speed) {
        RCLCPP_WARN(this->get_logger(), "Were attempting to go beyond max braking speed! Speed value received: %f",
                    msg.acceleration);
        msg.acceleration = _max_braking_speed; //TODO IMO we shouldn't overwrite the values here, since this is supposed to be odom, the state of the vehicle. Data logger will already drop invalid commands, so this will force us to record bad data, since sim will still execute the bad command
    } else if (msg.acceleration > _max_throttle_speed) {
        RCLCPP_WARN(this->get_logger(), "Were attempting to go beyond max throttle speed! Speed value received: %f",
                    msg.acceleration);
        msg.acceleration = _max_throttle_speed;
    } else if (msg.steering_angle > _max_steering_rad || msg.steering_angle < (-1 * _max_steering_rad)) {
        RCLCPP_WARN(this->get_logger(), "Were attempting to go beyond max steering angle! Value received: %f",
                    msg.steering_angle);
        msg.steering_angle = _max_steering_rad;
    }
}
