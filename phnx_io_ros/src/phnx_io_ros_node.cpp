#include <rclcpp/rclcpp.hpp>

#include "phnx_io_ros/phnx_io_ros.hpp"

pir::PhnxIoRos::PhnxIoRos(rclcpp::NodeOptions options) : Node("phnx_io_ros", options) {
    this->_port_pattern = this->declare_parameter("port_search_pattern", "/dev/ttyACM*");
    this->_baud_rate = this->declare_parameter("baud_rate", 115200);
    this->_max_throttle_speed = this->declare_parameter("max_throttle_speed", 2.0);
    this->_max_brake_speed = this->declare_parameter("max_brake_speed", 2.0);
    this->current_device = 0;

    // Wall timer to continuously read the current port with
    read_timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&PhnxIoRos::read_data, this));

    _odom_acks_pub = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("/odom_ack", 10);
    _acks_sub = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
        "/ack_vel", 10, std::bind(&PhnxIoRos::send_can_cb, this, std::placeholders::_1));
    _robot_state_client = this->create_client<robot_state_msgs::srv::SetState>("/robot/set_state");

    port = serial::serial(this->get_logger());

    // Find ports connected with the specified pattern
    port.find_ports(_port_pattern);

    //Connect every port that we found with the specific pattern
    for (const auto& i : port.get_ports()) {
        port.connect(i.port_name, _baud_rate);
        //if the port we just connected on successfully connected then enter that fd as the current device number to use
        if(i.port_number != -1 && current_device != -1){
            current_device = i.port_number;
        }
    }

    if (port.get_ports().size() < 2) {
        RCLCPP_WARN(this->get_logger(), "Only one device found! Automated fail-over not available!!!");
        fail_over_enabled = false;
    } else {
        fail_over_enabled = true;
    }
}

void pir::PhnxIoRos::send_can_cb(ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
    // Copy the newest message into last_ack and move speed field into
    // acceleration field
    ackermann_msgs::msg::AckermannDrive pub_msg;
    pub_msg.acceleration = msg->speed;
    pub_msg.steering_angle = msg->steering_angle;
    if (!can_msgs.empty()) {
        pub_msg.speed = can_msgs.front().data[0];
        can_msgs.pop_front();
    } else {
        pub_msg.speed = 0.0;
    }
    _odom_acks_pub->get()->publish(pub_msg);

    serial::message ser_msg{};
    ser_msg.length = 1;

    // Get percentage brake and throttle and send their respective messages
    if (msg->speed < 0) {
        auto percent_brake = static_cast<uint8_t>((abs(msg->speed) / _max_brake_speed) * 100);
        ser_msg.type = pir::CanMappings::SetBrake;
        ser_msg.data[0] = percent_brake;
    } else {
        auto percent_throttle = static_cast<uint8_t>((msg->speed / _max_throttle_speed) * 100);
        ser_msg.type = pir::CanMappings::SetThrottle;
        ser_msg.data[0] = percent_throttle;
    }
    RCLCPP_INFO(this->get_logger(), "Attempting to send message with type: %u, data: %u", ser_msg.type,
                ser_msg.data[0]);

    if (serial::serial::write_packet(current_device, reinterpret_cast<uint8_t*>(&ser_msg),
                          sizeof(serial::message)) == static_cast<uint8_t>(-1)) {
        // We failed a write so we need to check and see if fail-over is enabled
        RCLCPP_ERROR(this->get_logger(), "Failed to write message to teensy device! using fd: %d",
                     current_device);
        auto_fail_over();
    }

    // send steering angle message
    ser_msg.type = pir::CanMappings::SetAngle;
    ser_msg.data[0] = static_cast<uint8_t>(msg->steering_angle);
    RCLCPP_INFO(this->get_logger(), "Attempting to send message with type: %u, data: %u", ser_msg.type,
                ser_msg.data[0]);

    if (serial::serial::write_packet(current_device, reinterpret_cast<uint8_t*>(&ser_msg),
                          sizeof(serial::message)) == static_cast<uint32_t>(-1)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write message to teensy device!");
        auto_fail_over();
    }
}

void pir::PhnxIoRos::read_data() {
    if (serial::serial::read_packet(current_device, &read_buf, sizeof(serial::message)) ==
        static_cast<uint8_t>(-1)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read message from teensy device using fd: %d",
                     port.get_ports().at(0).port_number);
    } else {
        auto* msg = reinterpret_cast<serial::message*>(&read_buf);
        switch (msg->type) {
            case CanMappings::KillAuton:
                RCLCPP_WARN(this->get_logger(), "Received auton_kill message!");
                request->state.state = robot_state_msgs::msg::State::KILL;
                _robot_state_client->async_send_request(request);
                break;
            case CanMappings::EncoderTick:
                RCLCPP_INFO(this->get_logger(), "Received encoder message!");
                if (can_msgs.size() > 15) {
                    can_msgs.clear();
                }
                can_msgs.push_back(*msg);
                break;
        }
    }
}

void pir::PhnxIoRos::auto_fail_over() {
    //TODO: Implement auto fail over
    if(fail_over_enabled){
        //Fail-over is available so attempt to find another teensy device connected to the computer to use
        used_ports.push_back(current_device);
    }
    else{
        RCLCPP_FATAL(this->get_logger(), "We've lost connection to the Teensy device!");
        request->state.state = robot_state_msgs::msg::State::KILL;
        _robot_state_client->async_send_request(request);
    }
}

pir::PhnxIoRos::~PhnxIoRos() {
    // Clean up serial connection
    for (const auto& i : port.get_ports()) {
        port.close_connection(i.port_number);
    }
}