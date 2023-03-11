#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "gz_io_ros/gz_io_ros.hpp"

TEST(GzIoRosTests, QueueTest) {
    rclcpp::NodeOptions opts;

    gir::GzIoRos node{opts};


}

TEST(GzIoRosTests, QueueBoundingTest){
    //Test that twist/odom queue is bounded correctly
    rclcpp::NodeOptions opts;

    gir::GzIoRos node{opts};

}

TEST(GzIoRosTests, ZeroTwistOdomPopTest){
    //Test that when no twist exists in the queue, that the newest odom message and a zero twist message are sent out
    rclcpp::NodeOptions opts;

    gir::GzIoRos node{opts};
}

TEST(GzIoRosTests, TwistOdomPopTest){
    //Make sure that newest message is popped from each queue
    rclcpp::NodeOptions opts;

    gir::GzIoRos node{opts};
}

TEST(GzIoRosTests, MsgConvertTest){
    //Test that output ackermann messages are being popped correctly
    rclcpp::NodeOptions opts;

    gir::GzIoRos node{opts};
}

int main(int argc, char** argv) {
    rclcpp::init(0, nullptr);

    ::testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();

    rclcpp::shutdown();
    return res;
}
