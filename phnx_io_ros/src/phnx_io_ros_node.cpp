#include <cmath>
#include <rclcpp/rclcpp.hpp>

#include "phnx_io_ros/phnx_io_ros.hpp"

pir::PhnxIoRos::PhnxIoRos(rclcpp::NodeOptions options)
    : Node("phnx_io_ros", options), roboteq(this->declare_parameter<float>("motor_scale", 0.2)) {
    this->_port_pattern =
        this->declare_parameter("port_search_pattern", "/dev/serial/by-id/usb-Teensyduino_USB_Serial*");
    this->_baud_rate = this->declare_parameter("baud_rate", 115200);

    _odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom_can", 10);
    _acks_sub = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
        "/robot/ack_vel", 10, std::bind(&PhnxIoRos::send_can_cb, this, std::placeholders::_1));
    _filtered_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&PhnxIoRos::filtered_odom_cb, this, std::placeholders::_1));
    _robot_state_client = this->create_client<robot_state_msgs::srv::SetState>("/robot/set_state");

    // Connect to roboteq over USB
    while (!this->roboteq.connect()) {
        // RCLCPP_INFO(this->get_logger(), "Could not connect to roboteq!");
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }
    RCLCPP_INFO(this->get_logger(), "Connected to Roboteq!");

    // Check voltage on a timer
    // this->voltage_timer = this->create_wall_timer(std::chrono::seconds{1}, [this]() {
    //     // Only measure voltage when not killed, as killing the bot will cause the voltage to drop
    //     if (!this->killed) {
    //         try {
    //             auto maybe_voltage = this->roboteq.get_batt_voltage();

    //             if (maybe_voltage) {
    //                 RCLCPP_INFO(this->get_logger(), "Voltage: %f", *maybe_voltage);  //TODO pub sound if low
    //             } else {
    //                 RCLCPP_ERROR(this->get_logger(), "Failed read!");
    //             }
    //         } catch (std::system_error& e) {
    //             RCLCPP_ERROR(this->get_logger(), "Failed read with %s", e.what());
    //         }
    //     }
    // });

    // Find connected interface ECU connected to a USB port
    while (find_devices() != 0) {
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    //Create a link to the callback function to process new incoming messages
    std::function<void(serial::message)> read_callback = [this](auto&& PH1) {
        read_data(std::forward<decltype(PH1)>(PH1));
    };

    // Start CAN read thread
    cur_device.handler = new serial::serial(this->get_logger(), read_callback);

    while (cur_device.handler->open_connection(cur_device.port_name, this->_baud_rate) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error opening device!");
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    RCLCPP_INFO(this->get_logger(), "Connected to device!");

    // Start pid thread
    this->pid = std::make_unique<PidInterface>(std::bind(&PhnxIoRos::handle_pid_update, this, std::placeholders::_1));

    /* Now we have three threads:
     * 1) Main node thread subs and pubs, as well as Roboteq voltage
     * 2) CAN read thread
     * 3) PID control thread, which writes to CAN
     */
}

int pir::PhnxIoRos::find_devices() {
    // DO NOT INITIALIZE THIS WILL BREAK GLOB
    glob64_t gstruct;

    int result = glob64(_port_pattern.c_str(), GLOB_ERR, NULL, &gstruct);

    // Ensure we actually found a serial port
    if (result != 0) {
        if (result == GLOB_NOMATCH) {
            RCLCPP_ERROR(this->get_logger(), "Failed to find serial device using search pattern!");
            return -1;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unknown glob error!");
            return -1;
        }
    }

    // Set first found device as our current device
    this->cur_device.port_name = *gstruct.gl_pathv;
    return 0;
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void pir::PhnxIoRos::send_can_cb(ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
    // If killed, make sure we avoid updating the pid, so it says at 0

    // RCLCPP_INFO(this->get_logger(), "TEST TEST");
    if (this->killed) {
        return;
    }

    // Send speed command to PID
    this->pid->set_command(*msg);

    serial::steer_msg st_msg{};
    st_msg.type = pir::CanMappings::SetAngle;

    // ROS steering is in rad, bus is in degrees
    st_msg.angle = msg->steering_angle / M_PI * 180;

    st_msg.angle = std::clamp(st_msg.angle, -15.66f, 15.66f);

    /* Found with arctan(1.08/3.85), where 1.08m is the wheelbase, and 3.85m is the turning radius. This leads to phnx
       Having a max steering angle of 15 degrees or so. 24 is just the max on the ST board, which linearly maps the range.
       This may not be accurate (the steering range may not be linear), but is a decent approximation
    */
    st_msg.angle = mapfloat(st_msg.angle, -15.66, 15.66, -24, 24);

    {
        std::unique_lock lk{this->last_steering_mtx};
        this->last_steering_angle = st_msg.angle;
    }

    // Send a steering message to the interface device to publish onto the CAN bus
    // RCLCPP_INFO(this->get_logger(), "Sending steer msg with angle: %f", st_msg.angle);
    if (cur_device.handler->write_packet(reinterpret_cast<uint8_t*>(&st_msg), sizeof(serial::steer_msg)) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write message to device: %s with fd: %d",
                     cur_device.port_name.c_str(), cur_device.handler->get_fd());
    }
}

// Keep in mind, this occurs in the read thread
void pir::PhnxIoRos::read_data(serial::message m) {
    auto kill = std::make_shared<robot_state_msgs::srv::SetState::Request>();
    serial::enc_msg* msg;

    nav_msgs::msg::Odometry odom{};
    switch (m.type) {
        case CanMappings::KillAuton:
            RCLCPP_WARN(this->get_logger(), "Auton kill signal received!");
            kill->state.state = robot_state_msgs::msg::State::KILL;
            this->_robot_state_client->async_send_request(kill);

            this->killed = true;
            // Set PID to 0 set speed on estop, to avoid accumulating error when stopped.
            this->pid->set_command(ackermann_msgs::msg::AckermannDrive{});

            // Also set roboteq to zero, so it actually stops, since PID will no longer send commands
            try {
                bool res = this->roboteq.set_power(0);

                if (res) {
                    RCLCPP_ERROR(this->get_logger(), "Roboteq responded with non + !");
                }
            } catch (std::system_error& error) {
                RCLCPP_ERROR(this->get_logger(), "Writing to Roboteq failed with: %s", error.what());
            }

            break;

        case CanMappings::EnableAuton:
            RCLCPP_INFO(this->get_logger(), "Auton enable signal received!");

            // Wait for brake to unclamp, so we don't stall the motor. This also gives the steering time to home.
            // This wait allows for us to avoid having silly "wait 5 seconds" in our SOP.
            rclcpp::sleep_for(std::chrono::seconds{10});

            this->killed = false;

            // Send enable transition
            kill->state.state = robot_state_msgs::msg::State::ACTIVE;
            this->_robot_state_client->async_send_request(kill);
            break;

        case CanMappings::EncoderTick:
            // Publish odom from encoder
            msg = reinterpret_cast<serial::enc_msg*>(m.data);

            {
                std::unique_lock lk{this->last_steering_mtx};

                odom.twist.twist.linear.x = msg->speed;
                //                odom.twist.twist.linear.y = 0;
                odom.header.stamp = this->get_clock()->now();

                //                odom.twist.covariance.at(0) = TODO in theory these lines represent the encoder better, needs tuning
                //                    0.05 * std::abs(last_steering_angle) + 0.001;  // x has more error when turning
                //                odom.twist.covariance.at(7) = 0.001;               // y, we cannot move in y
            }

            this->_odom_pub->publish(odom);
            break;
        default:
            RCLCPP_INFO(this->get_logger(), "Received non encoder tick/auton kill message!");
            break;
    }
}

void pir::PhnxIoRos::filtered_odom_cb(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    // If killed, make sure we avoid updating the pid, so it says at 0
    if (this->killed) {
        return;
    }

    this->pid->add_feedback(*msg);
    // RCLCPP_INFO(this->get_logger(), "Speed: %f", msg->twist.twist.linear.x);
}

void pir::PhnxIoRos::handle_pid_update(std::tuple<double, phnx_control::SpeedController::Actuator> control) {
    // This shouldn't be possible, but avoid sending commands when killed as a sanity check
    if (this->killed) {
        return;
    }

    auto [val, actuator] = control;

    // RCLCPP_INFO(this->get_logger(), "Sending drive msg with level: %f and actuator: %u", val, uint32_t(actuator));

    serial::drive_msg throttle{};
    serial::drive_msg brake{};

    // Fit to limits
    val = std::clamp(val, -1.0, 1.0);

    if (actuator == phnx_control::SpeedController::Actuator::Throttle) {
        // Set throttle to control, and zero brake
        throttle.type = CanMappings::SetThrottle;
        throttle.speed = uint8_t(std::abs(val) * 100);

        brake.type = CanMappings::SetBrake;
        brake.speed = 0;

        // Send commands to can, only the brakes use can *
        // RCLCPP_INFO(this->get_logger(), "Sending throttle command: %f", val);
        // actual power demand no longer handled thru can, kept just in case.
        auto wrt_res = this->cur_device.handler->write_packet(reinterpret_cast<uint8_t*>(&throttle), sizeof(throttle));
        wrt_res += this->cur_device.handler->write_packet(reinterpret_cast<uint8_t*>(&brake), sizeof(brake));

        if (wrt_res < 0) {
            RCLCPP_ERROR(this->get_logger(), "Writing to interface failed!");
        }

        // Send command to Roboteq
        try {
            bool res = this->roboteq.set_power(float(val));

            if (res) {
                // RCLCPP_ERROR(this->get_logger(), "Roboteq responded with non + !");
            }
        } catch (std::system_error& error) {
            RCLCPP_ERROR(this->get_logger(), "Writing to Roboteq failed with: %s", error.what());
        }
    } else {
        // Set brake to control, and zero throttle
        throttle.type = CanMappings::SetThrottle;
        throttle.speed = 0;

        brake.type = CanMappings::SetBrake;
        brake.speed = uint8_t(std::abs(val) * 100);

        // Send commands to can
        RCLCPP_INFO(this->get_logger(), "Sending brake command: %f", val);
        this->cur_device.handler->write_packet(reinterpret_cast<uint8_t*>(&throttle), sizeof(throttle));
        // previous line sends can message, but roboteq (and commanded speed) is not handled thru can
        this->roboteq.set_power(0.0f);
        this->cur_device.handler->write_packet(reinterpret_cast<uint8_t*>(&brake), sizeof(brake));
    }
}

void pir::PhnxIoRos::close() {
    // Clean up serial connection
    if (cur_device.handler->connected()) {
        cur_device.handler->close_connection();
    }
}

pir::PhnxIoRos::~PhnxIoRos() { close(); }