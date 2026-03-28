#include "phnx_control/speed_control.hpp"

namespace phnx_control {

SpeedController::SpeedController(double p, double i, double d, double max_i, double min_i, bool antiwindup) {
    this->throttle_pid.initPid(p, i, d, max_i, min_i, antiwindup);
}

std::tuple<double, SpeedController::Actuator> SpeedController::update(double speed, const rclcpp::Time& stamp) {
    uint64_t dt;
    if (!this->last_feedback.has_value()) {
        dt = 0;
    } else {
        dt = (stamp - *this->last_feedback).nanoseconds();
    }
    this->last_feedback = stamp;

    // Run PID
    double error = this->set_speed - speed;
    double command = this->throttle_pid.computeCommand(error, dt);  // TODO attempt to use brake if large decel needed

    this->last_command = command;
    this->last_speed = speed;

    return std::make_tuple(command, Actuator::Throttle);
}

void SpeedController::update_set_speed(double speed) { this->set_speed = speed; }

std::tuple<double, double, double, double, double> SpeedController::get_components() {
    double p, i, d;
    this->throttle_pid.getCurrentPIDErrors(p, i, d);

    return std::make_tuple(p, i, d, this->set_speed, this->last_speed);
}

}  // namespace phnx_control
