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
kart state based on CAN bus status.

### Topics

#### Publishes

- `/odom_ack` : AckermannDrive message that contains the current steering angle received on the topic `/robot/cmd_vel`
  as well as the current throttle/brake value which will be stored in the resulting messages acceleration field.
  The speed field of the AckermannDrive message contains the last received speed message from the CAN bus.
  If no speed messages have been received, then the speed field will be zero.

#### Subscribes

- `/robot/ack_vel`: AckermannDrive messages coming from the controller that contain steering angle and throttle/brake
  values to be sent to the CAN bus.

### Params

- `port_search_pattern`: The string pattern to look for when looking for a teensy device. This can be either a top level
  `/dev/ttyACM*` or the serial devices id as found in `/dev/serial/by-id`.
  By default, phnx_io_ros will use `/dev/serial/by-id/usb-Teensyduino_USB_Serial*` which should point to a Teensy serial
  monitor
  if plugged in.
- `baud_rate`: Baud rate to use when connecting to a Teensy.
- `max_throttle_speed`: Value that we consider to be full throttle. It Should be consistent with controller.
- `max_braking_speed`: Value that we consider to be full brake. It Should be consistent with controller.

### gz_io_ros

This package acts as a fake for phnx_io_ros and allows the ROS aspect of Phoenix to interface with Ignition Gazebo,
which
runs our simulation environment. This packages primary function is to take in odom coming from the simulation and twist
values from ROS and convert those into AckermannDrive messages to be logged
with '[data_logger](https://github.com/ISC-Project-Phoenix/data_logger)'

### Topics

#### Publishes

- `/odom_ack`: AckermannDrive messages that contain the current steering, speed and throttle/brake percentage. Steering
  values and speed are stored in their respective fields; however, the acceleration field stores either the current
  braking
  percentage or current throttle percentage depending on the received twist message. Note that
  the `steering_angle_velocity`
  and `jerk` fields are unused.

#### Subscribes

- `/odom`: Odom data coming from the simulation
- `/robot/cmd_vel`: Twist data coming from the controller

### Params

- `wheel_base`: Wheelbase of the kart, it should match both sim and real life

### Misc

see [the design for more info](https://github.com/ISC-Project-Phoenix/design/blob/main/software/ros/gz_io_ros.md)

### wb_io_ros

This package contains a webots plugin that simulates the phoenix can bus, like gz_io_ros. This bridges any webots
sensors that have no default plugins, and also exposes an interface for the actuation of AckermannDrive messages.
See the tutorial for an example on how to use
this: [tutorial](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Pluginlib.html).

### Topics

#### Publishes

- `/odom_can`: Encoder data from webots

#### Subscribes

- `/ack_vel`: AckermannDrive commands to actuate
