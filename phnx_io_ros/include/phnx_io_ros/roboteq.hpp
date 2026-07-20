#pragma once

#define termios asmtermios
#include <CppLinuxSerial/SerialPort.hpp>
#undef termios

#include <functional>
#include <memory>
#include <regex>
#include <vector>

#include "glob.h"
#include "optional"

/// Interface to a Roboteq motor controller connected over USB. Threadsafe through internal locking.
/// If the device disconnects mid-command, set_power throws so the caller can log it, and
/// reconnection is attempted on the next command.
class Roboteq {
    std::unique_ptr<mn::CppLinuxSerial::SerialPort> serial;
    /// 0-1 float to scale all commands by. Caps the speed.
    float power_scale;
    /// Serial port mtx
    std::mutex mtx;

    /// Finds the usb port for the roboteq
    std::optional<std::string> enumerate_port() {
        // DO NOT INITIALIZE THIS WILL BREAK GLOB
        glob64_t gstruct;

        int result = glob64("/dev/serial/by-id/usb-RoboteQ_*", GLOB_ERR, NULL, &gstruct);

        // Ensure we actually found a serial port
        if (result != 0) {
            if (result == GLOB_NOMATCH) {
                return std::nullopt;
            } else {
                return std::nullopt;
            }
        }

        // Set first found device as our current device
        return std::string{*gstruct.gl_pathv};
    }

    /// Checks if the roboteq response indicates ok.
    bool check_msg_ok(const std::string& s) { return s.find('+') != std::string::npos; }

    /// True if the serial port is currently open. Reconnect if this is false.
    bool port_open() { return this->serial && this->serial->GetState() == mn::CppLinuxSerial::State::OPEN; }

public:
    explicit Roboteq(float power_scale) { this->power_scale = power_scale; }

    /// Connects to the roboteq over USB.
    bool connect() {
        auto maybe_port = this->enumerate_port();

        if (!maybe_port) {
            return false;
        }

        try {
            this->serial = std::make_unique<mn::CppLinuxSerial::SerialPort>(
                *maybe_port, mn::CppLinuxSerial::BaudRate::B_115200, mn::CppLinuxSerial::NumDataBits::EIGHT,
                mn::CppLinuxSerial::Parity::NONE, mn::CppLinuxSerial::NumStopBits::ONE);

            this->serial->SetTimeout(50);
            this->serial->Open();
        } catch (const std::exception& e) {
            // Open throws if the port exists but can't be used yet (permissions, still enumerating)
            this->serial.reset();
            return false;
        }

        return this->serial->GetState() == mn::CppLinuxSerial::State::OPEN;
    }

    /// Sets the motor to the percent power, governed by power scaling.
    /// Returns false if the roboteq NAKs or cannot be found. Throws if the device
    /// disconnects mid-command; the next call will reconnect.
    bool set_power(float percent) {
        // -1 <= percent <= 1

        std::unique_lock lk{mtx};

        // Reestablish the connection if the device dropped
        if (!this->port_open() && !this->connect()) {
            return false;
        }

        // Scale output
        // Might need to lower this with the new ESC (maybe 250? -berto)

        int16_t level = percent * 1000 * this->power_scale;  // power_scale currently 0.2
        level = std::clamp(level, int16_t(-1000), int16_t(1000));
        // via
        // https://www.scribd.com/document/832439076/Roboteq-Controllers-User-Manual-v3-2-225-488
        // "!G 1 'x' _", -1000 <= x <= 1000, we just never demanded 1000 so this was never an issue

        try {
            // Send go command to percent max power
            this->serial->Write(std::string{"!G 1 " + std::to_string(level) + " _"});

            std::string res{};
            this->serial->Read(res);

            return this->check_msg_ok(res);
        } catch (const std::exception& e) {
            // Write/Read throw if the device disconnected. Drop the port so the next call
            // reconnects, then rethrow so the caller can log what happened
            this->serial.reset();
            throw;
        }
    }

    /// Send a ping to the Roboteq, and check for its response.
    bool check_alive() {
        std::unique_lock lk{mtx};

        // Reestablish the connection if the device dropped
        if (!this->port_open() && !this->connect()) {
            return false;
        }

        try {
            // Send ENQ
            std::vector<uint8_t> vec{0x5};
            this->serial->WriteBinary(vec);

            vec.clear();
            this->serial->ReadBinary(vec);

            // ACK
            return !vec.empty() && vec[0] == 0x6;
        } catch (const std::exception& e) {
            this->serial.reset();
            return false;
        }
    }

    /// Get battery voltage, if possible.
    std::optional<float> get_batt_voltage() {
        std::unique_lock lk{mtx};

        // Reestablish the connection if the device dropped
        if (!this->port_open() && !this->connect()) {
            return std::nullopt;
        }

        std::string res{};
        try {
            // Ask for battery voltage
            this->serial->Write(std::string{"?V 2 _"});
            this->serial->Read(res);
        } catch (const std::exception& e) {
            this->serial.reset();
            return std::nullopt;
        }

        // Should be V=XXX, where XXX is voltage*10
        if (res.size() > 3) {
            auto volts_str = res.substr(2, 3);

            // A lot of things can go wrong, so probe in the right direction
            try {
                float volts = std::stof(volts_str) / 10.0f;

                if (volts > 40 && volts < 60) {
                    return volts;
                } else {
                    return std::nullopt;
                }
            } catch (std::invalid_argument& e) {
                return std::nullopt;
            }
        }

        return std::nullopt;
    }
};