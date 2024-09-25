// This header file will be used for serial communication between motors and the hardware_interface

#ifndef ROS2_CONTROL_DEMO_EXAMPLE_2__SERIAL_COMMS_HPP_
#define ROS2_CONTROL_DEMO_EXAMPLE_2__SERIAL_COMMS_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "diffdrive_arduino/visibility_control.h"