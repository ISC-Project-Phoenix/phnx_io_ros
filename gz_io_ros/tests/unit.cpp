#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "gz_io_ros/gz_io_ros.hpp"

TEST(GzIoRosTests, MsgConvertTest) {
    //Test that output ackermann messages are being converted correctly
    rclcpp::NodeOptions opts;
    gir::GzIoRos node{opts};

    geometry_msgs::msg::Twist::SharedPtr t{};
    nav_msgs::msg::Odometry::SharedPtr o{};
    t->linear.x = 5.0;
    t->angular.z = 1.57;
    o->twist.twist.linear.x = 2.5;
    node.convert_data(o, t);

}

TEST(GzIoRosTests, MsgValidateTest){
    rclcpp::NodeOptions opts;
    gir::GzIoRos node{opts};
    float VALID_ANGLE = 1.57;
    float VALID_SPEED = 5.0;
    float VALID_ACCEL = 5.0;
    float INVALID_ANGLE = 30.0;
    float INVALID_SPEED = 30.0;
    float INVALID_ACCEL = 30.0;
    ackermann_msgs::msg::AckermannDrive msg;
    msg.steering_angle = VALID_ANGLE;
    msg.acceleration = VALID_SPEED;
    msg.speed = VALID_ACCEL;
    node.validate_msg(msg);
    EXPECT_EQ(msg.steering_angle, VALID_ANGLE);
    EXPECT_EQ(msg.speed, VALID_SPEED);
    EXPECT_EQ(msg.acceleration, VALID_ACCEL);

}

int main(int argc, char** argv) {
    rclcpp::init(0, nullptr);

    ::testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();

    rclcpp::shutdown();
    return res;
}
