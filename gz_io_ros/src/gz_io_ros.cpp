#include "gz_io_ros/gz_io_ros.hpp"

#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto gir_node = std::make_shared<gir::GzIoRos>(options);
    exec.add_node(gir_node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
