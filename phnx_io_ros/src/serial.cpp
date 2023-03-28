#include "phnx_io_ros/serial.hpp"

#include <glob.h>

#include <iostream>

serial::serial::serial(rclcpp::Logger logger) { this->log = &logger; }

int serial::serial::find_ports(const std::string& pattern) {
    // DO NOT INITIALIZE THIS WILL BREAK GLOB
    glob64_t gstruct;

    int result = glob64(pattern.c_str(), GLOB_ERR, NULL, &gstruct);

    // Ensure we actually found a serial port
    if (result != 0) {
        if (result == GLOB_NOMATCH) {
            logger("Failed to find serial device using search pattern!", -2);
            return -1;
        } else {
            logger("Unknown glob error!", -2);
            return -2;
        }
    }

    while (*gstruct.gl_pathv) {
        ports.push_back({*gstruct.gl_pathv, -1});
        gstruct.gl_pathv++;
    }
    return 0;
}

std::list<serial::port_info> serial::serial::get_ports() { return this->ports; }

int serial::serial::connect(std::string str) {
    int result = open(str.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

    if (result < 0) {
        logger("Error opening serial connection!", -2);
        return -1;
    }

    // Store port file descriptor with string name
    for (auto i : this->ports) {
        if (strcmp(str.c_str(), i.port_name.c_str()) == 0) {
            i.port_number = result;
            break;
        }
    }
    return result;
}

int serial::serial::configure(int port_num, long baud) {
    speed_t termios_baud;
    switch (baud) {
        case 9600:
            termios_baud = B9600;
            break;
        case 115200:
            termios_baud = B115200;
            break;
        case 1152000:
            termios_baud = B1152000;
            break;
        default:
            termios_baud = B115200;
            break;
    }
    // Get params from port
    if (tcgetattr(port_num, &tty) != 0) {
        logger("Error from tcgetattr!", -2);
        return -1;
    }
    // Set port parameters
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CS8;  // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;    // Disable echo
    tty.c_lflag &= ~ECHOE;   // Disable erasure
    tty.c_lflag &= ~ECHONL;  // Disable new-line echo
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~ONLCR;
    tty.c_oflag &= ~OPOST;

    // Set baud rate
    cfsetispeed(&tty, termios_baud);

    if (tcsetattr(port_num, TCSANOW, &tty) != 0) {
        logger("Error from tcsetattr!", -2);
        return -1;
    }
    return 0;
}

uint32_t serial::serial::read_packet(int port_num, uint8_t* buf, uint32_t length) {
    return read(port_num, buf, length);
}

uint32_t serial::serial::write_packet(int port_num, uint8_t* buf, uint32_t length) {
    return write(port_num, buf, length);
}

void serial::serial::close_connection(int port_num) const {
    int result = close(port_num);
    if (result != 0) {
        logger("Failed to properly close connection!", -1);
    }
}

void serial::serial::logger(const std::string& str, int severity) const {
    if (this->log) {
        switch (severity) {
            case 1:
                RCLCPP_WARN(*(this->log), "%s", str.c_str());
                break;
            case 0:
                RCLCPP_INFO(*(this->log), "%s", str.c_str());
                break;
            case -1:
                RCLCPP_ERROR(*(this->log), "%s", str.c_str());
                break;
            case -2:
                RCLCPP_FATAL(*(this->log), "%s", str.c_str());
                break;
            default:
                RCLCPP_INFO(*(this->log), "%s", str.c_str());
                break;
        }
    } else {
        if (severity < 0) {
            fprintf(stderr, "%s", str.c_str());
        } else {
            fprintf(stdout, "%s", str.c_str());
        }
    }
}
