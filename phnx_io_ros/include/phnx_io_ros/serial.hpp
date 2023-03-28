#pragma once

#include <fcntl.h>  // Contains file controls like O_RDWR
#include <glob.h>
#include <libudev.h>
#include <termios.h>  // Contains POSIX terminal control definitions
#include <unistd.h>   // write(), read(), close()

#include <cerrno>  // Error integer and strerror() function
#include <cstdint>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace serial {

struct message {
    uint8_t type;
    uint16_t length;
    uint8_t data[512];
} __attribute__((packed));

struct enc_msg {
    uint16_t ticks;
    float speed;
} __attribute__((packed));

/// Contains the name of the serial port file and the file descriptor of the
/// port, if device on port not connected then port_number will be -1
struct port_info {
    std::string port_name;
    int port_number;
};

class serial {
private:
    struct termios tty {};
    rclcpp::Logger* log = nullptr;
    std::list<port_info> ports;

    /// Logs with either RCLCPP logs or normal stdout/stderr
    ///@param str string to write to the log
    ///@param severity severity of the log, 1 is warning, 0 is info, -1 is error,
    ///and -2 is fatal
    void logger(const std::string& str, int severity) const;

public:
    serial() = default;

    explicit serial(rclcpp::Logger log);

    /// Find serial ports using a given pattern
    ///@param pattern string pattern to use to for search
    int find_ports(const std::string& pattern);

    /// Connect and configure a serial port
    ///@param str Name of a port to connect to
    ///@param baud baud rate to use
    int connect(std::string port_name);

    /// Configure a serial port
    ///@param port_num file descriptor for a connected port
    ///@param baud baud rate to use
    int configure(int port_num, long baud);

    /// Closes connection to a serial port
    ///@param port_num file descriptor for a connected port
    void close_connection(int port_num) const;

    /// Get the list of found serial ports, port number will be -1 if port is not
    /// connected
    ///@return Returns a list of port info structs containing string filename of
    ///the port and file descriptor used by termios
    std::list<port_info> get_ports();

    /// Read data from a connected serial port
    ///@param buf buffer to store read data in
    ///@param length length of data to read
    ///@param port_num file descriptor for a connected port
    ///@return number of bytes read, -1 returned on error
    static uint32_t read_packet(int port_num, uint8_t* buf, uint32_t length);

    /// Write data to a connected serial port
    ///@param buf data to write to the port
    ///@param length length of data
    ///@param port_num file descriptor for a connected port
    ///@return number of bytes written, -1 returned on error
    static uint32_t write_packet(int port_num, uint8_t* buf, uint32_t length);
};
}  // namespace serial