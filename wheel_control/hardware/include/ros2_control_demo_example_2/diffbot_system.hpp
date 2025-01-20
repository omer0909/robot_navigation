// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_CONTROL_DEMO_EXAMPLE_2__DIFFBOT_SYSTEM_HPP_
#define ROS2_CONTROL_DEMO_EXAMPLE_2__DIFFBOT_SYSTEM_HPP_

#include <gpiod.h>

#include <atomic>
#include <memory>
#include <string>
#include <thread>
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

namespace ros2_control_demo_example_2 {
class GpiodPidController {
 public:
  void set_vel_r(double vel);
  void set_vel_l(double vel);
  int64_t get_pos_r();
  int64_t get_pos_l();
  GpiodPidController();
  ~GpiodPidController();

 private:
  std::atomic_bool active = true;
  std::atomic_int64_t pos_l = 0;
  std::atomic_int64_t pos_r = 0;

  void encoder_listener();
  void pid_controller();

  std::thread encoder_listener_thread;
  std::thread pid_controller_thread;
  gpiod_chip* chip;
};

class DiffBotSystemHardware : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware);

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  // Parameters for the DiffBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
//   GpiodPidController wheel_driver;
};

}  // namespace ros2_control_demo_example_2

#endif  // ROS2_CONTROL_DEMO_EXAMPLE_2__DIFFBOT_SYSTEM_HPP_
