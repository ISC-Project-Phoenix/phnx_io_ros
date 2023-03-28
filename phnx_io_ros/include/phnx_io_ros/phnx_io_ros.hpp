#pragma once

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "optional"
#include "phnx_io_ros/serial.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_state_msgs/srv/set_state.hpp"

namespace pir {

enum CanMappings {
    KillAuton = 0x0,
    SetBrake = 0x1,
    LockBrake = 0x2,
    UnlockBrake = 0x3,
    SetAngle = 0x4,
    SetThrottle = 0x6,
    EncoderTick = 0x7,
    TrainingMode = 0x8,
};

class PhnxIoRos : public rclcpp::Node {
public:
    explicit PhnxIoRos(rclcpp::NodeOptions options);

    ~PhnxIoRos() override;

private:
    std::optional<std::shared_ptr<rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>>> _acks_sub = std::nullopt;

    std::optional<std::shared_ptr<rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>>> _odom_acks_pub =
        std::nullopt;

    rclcpp::Client<robot_state_msgs::srv::SetState>::SharedPtr _robot_state_client;

    rclcpp::TimerBase::SharedPtr read_timer_;

    // Serial port params
    serial::serial port;
    std::string _port_pattern{};
    std::vector<int> used_ports{};
    std::list<serial::enc_msg> enc_msgs;
    uint8_t read_buf{};
    long _baud_rate{};
    //File descriptor number of device were using, used to determine what device to write/read to
    int current_device{-1};

    // Kart control params
    double _max_throttle_speed{};
    double _max_brake_speed{};
    ackermann_msgs::msg::AckermannDrive last_ack{};
    robot_state_msgs::srv::SetState::Request::SharedPtr request =
        std::make_shared<robot_state_msgs::srv::SetState::Request>();

    ///@brief Convert ackermann messages into CAN messages and send them to the
    /// CAN bus
    ///@param msg Ackermann drive message to convert
    void send_can_cb(ackermann_msgs::msg::AckermannDrive::SharedPtr msg);

    ///@breif Reads data of size serial::message from connected port
    void read_data();

    ///@brief Handles automatically failing over to a second interface device if needed
    void reconnect();
};

}  // namespace pir