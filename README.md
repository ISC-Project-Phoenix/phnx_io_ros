# Phnx_Io_Ros

From package '[phnx_io_ros](https://github.com/ISC-Project-Phoenix/phnx_io_ros)'

# Files

- phnx_io_ros `./phnx_io_ros/src/phnx_io_ros.cpp`
- gz_io_ros `./gz_io_ros/src/gz_io_ros.cpp`

## Summary

Interface packages for Project Phoenix. Each of these is designed to allow the ROS aspect of Phoenix to interface with
either our simulation environment or the physical kart.

### phnx_io_ros

This package allows the ROS aspect of phoenix to interface with the CAN bus that will run the physical karts
drive-by-wire
system and receive CAN messages to translate to speed values. This package will also be responsible for controlling the
karts state based on CAN bus status.

### TODO: Rest of phnx_io_ros doc

### gz_io_ros

This package acts as a fake for phnx_io_ros and allows the ROS aspect of Phoenix to interface with Ignition Gazebo which
runs our simulation environment. This packages primary function is to take in odom coming from the simulation and twist
values from ROS and convert those into AckermannDrive messages to be logged
with '[data_logger](https://github.com/ISC-Project-Phoenix/data_logger)'

### Topics

#### Publishes

- `/odom_ack`: AckermannDrive messages that contain the current steering, speed and throttle/brake percentage. Steering
  values and speed are stored in their respective fields, however the acceleration field store either the current
  braking
  percentage or current throttle percentage depending on the received twist message. Note that
  the `steering_angle_velocity`
  and `jerk` fields are unused.

#### Subscribes

- `/odom`: Odom data coming from the simulation
- `/robot/cmd_vel`: Twist data coming from the controller

### Params

- `wheel_base`: Wheelbase of the kart, should match both sim and real life

### Misc

see [the design for more info](https://github.com/ISC-Project-Phoenix/design/blob/main/software/ros/gz_io_ros.md) 