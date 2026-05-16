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

//Returns the current error values of the PID loop
std::tuple<double, double, double, double, double> SpeedController::get_components() {
    double p, i, d;
    this->throttle_pid.getCurrentPIDErrors(p, i, d);

    return std::make_tuple(p, i, d, this->set_speed, this->last_speed);
}

//Returns the current set coeffients for the PID loop
std::tuple<double, double, double> SpeedController::get_coeffs() {
    double kp, ki, kd, umax, umin; 
    bool wind;
    this->throttle_pid.getGains(kp, ki, kd, umax, umin, wind);

    return std::make_tuple(kp, ki, kd);
}

//Sets coefficients for the PID loop (hopefully!)
void SpeedController::set_coeffs(double kp, double ki, double kd) {
    //Gets current values of the loop so we can just keep the values for the u and the antiwindup
    //Better ways to do this? totally. My apologies go out to whoever fixes this -Mark
    double old_p, old_i, old_d, umax, umin; 
    bool wind;
    this->throttle_pid.getGains(old_p, old_i, old_d, umax, umin, wind);
    
    //Actually sets the damn values
    this->throttle_pid.setGains(kp, ki, kd, umax, umin, wind);
}

}  // namespace phnx_control
