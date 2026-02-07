#pragma once

#include <optional>
#include <rclcpp/rclcpp.hpp>

#include "control_toolbox/pid.hpp"

namespace phnx_control {

/// Controls vehicle speed using throttle and brakes.
class SpeedController {
public:
    enum class Actuator { Brake, Throttle };

private:
    control_toolbox::Pid throttle_pid;
    /// Stamp of the last feedback
    std::optional<rclcpp::Time> last_feedback;
    /// Speed we want to reach
    double set_speed = 0.0;
    /// Last outputted command
    double last_command = 0.0;
    /// Last feedback speed
    double last_speed = 0.0;

public:
    explicit SpeedController(double p = 0.01, double i = 0.01, double d = 0, double max_i = 1, double min_i = 0,
                             bool antiwindup = true);

    /// Updates the controller, and creates a new command. The returned tuple contains a
    /// UNCAPPED value for the actuator (only igain is capped), as well as the enum initiating the actuator it applies to.
    std::tuple<double, Actuator> update(double speed, const rclcpp::Time& stamp);

    /// Sets a new set speed. If negative, will NOT be pinned to zero.
    void update_set_speed(double speed);

    /// Returns last (pe, ie, de, set speed, feedback)
    std::tuple<double, double, double, double, double> get_components();
};

}  // namespace phnx_control
