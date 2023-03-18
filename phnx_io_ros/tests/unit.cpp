#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "phnx_io_ros/phnx_io_ros.hpp"
#include "phnx_io_ros/serial.hpp"

TEST(phnx_io_ros, FillerTest) { EXPECT_EQ(2, 2); }

int main(int argc, char** argv) {
    rclcpp::init(0, nullptr);

    ::testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();

    rclcpp::shutdown();
    return res;
}