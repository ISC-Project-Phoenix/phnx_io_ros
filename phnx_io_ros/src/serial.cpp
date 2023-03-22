#include "phnx_io_ros/serial.hpp"

#include <glob.h>

#include <iostream>

serial::serial::serial(rclcpp::Logger logger) { this->log = &logger; }

void serial::serial::find_ports(const std::string& pattern) {
    // DO NOT INITIALIZE THIS WILL BREAK GLOB
    glob64_t gstruct;

    int result = glob64(pattern.c_str(), GLOB_ERR, NULL, &gstruct);

    // Ensure we actually found a serial port
    if (result != 0) {
        if (result == GLOB_NOMATCH) {
            logger("Failed to find serial device using search pattern!", -2);
            return;
        } else {
            logger("Unknown glob error!", -2);
            return;
        }
    }
    logger("Found ports using pattern!", 0);

    while (*gstruct.gl_pathv) {
        ports.push_back({*gstruct.gl_pathv, -1});
        gstruct.gl_pathv++;
    }
}

std::vector<serial::port_info> serial::serial::get_ports() { return this->ports; }

void serial::serial::connect(const std::string& str, long baud) {
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
    logger("Attempting to connect to port", 0);

    int result = open(str.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (result < 0) {
        logger("Error connecting to the serial port!", -2);
        return;
    }

    // Store port file descriptor with string name
    for (auto i : this->ports) {
        if (strcmp(str.c_str(), i.port_name.c_str()) == 0) {
            i.port_number = result;
            break;
        }
    }
    //this->ports.at(0).port_number = result;

    logger("Connected to serial port!", 0);
    configure(termios_baud, result);
}

void serial::serial::configure(speed_t baud, int port_num) {
    // Get params from port
    if (tcgetattr(port_num, &tty) != 0) {
        logger("Error from tcgetattr!", -2);
        return;
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
    cfsetispeed(&tty, baud);

    if (tcsetattr(port_num, TCSANOW, &tty) != 0) {
        logger("Error from tcsetattr!", -2);
        return;
    }
    logger("Serial port configured!", 0);
}

uint32_t serial::serial::read_packet(int port_num, char* buf, int length) {
    uint32_t len = read(port_num, buf, length);
    return len;
}

uint32_t serial::serial::write_packet(int port_num, uint8_t* buf, int length) {
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
                throw;
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
