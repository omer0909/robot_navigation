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

class PWM {
 public:
  enum Channel {
    Pwm0,
    Pwm1
  };

  PWM(Channel channel_);

  void set_duty(double duty);

  ~PWM();

 private:
  void writeToFile(const std::string& path, const std::string& value);

  std::string chip = "/sys/class/pwm/pwmchip2";
  std::string path;
  Channel channel;
};

class Motor{
  public:
  constexpr static double DUTY_MAX = 0.15;
  constexpr static int AS5600_ADDR = 0x36;
  constexpr static uint8_t AS5600_REG_RAW_ANGLE = 0x0C;
  constexpr static int AS5600_RES = 4096;

  constexpr static double STIFFNESS = 0.001;
  constexpr static double DAMPING = 0.000005;

  void set_vel_r(double vel);
  void set_vel_l(double vel);
  double get_pos_r();
  double get_pos_l();
  void set_duty_r(double duty);
  void set_duty_l(double duty);
  Motor();
  ~Motor();
  
  double target_pos_l = 0;
  double target_pos_r = 0;


  private:
  std::atomic_bool active = true;

  std::atomic_int64_t pos_l = 0;
  std::atomic_int64_t pos_r = 0;
  std::atomic<double> target_vel_l = 0;
  std::atomic<double> target_vel_r = 0;

  void encoder_listener_l();
  void encoder_listener_r();

  std::thread encoder_listener_thread_l;
  std::thread encoder_listener_thread_r;

  gpiod_chip* chip;
  gpiod_line* motor_dir_l;
  gpiod_line* motor_dir_r;
  
  PWM motor_pwm_l{PWM::Channel::Pwm1};
  PWM motor_pwm_r{PWM::Channel::Pwm0};
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
  Motor wheel_driver;
};

}  // namespace ros2_control_demo_example_2

#endif  // ROS2_CONTROL_DEMO_EXAMPLE_2__DIFFBOT_SYSTEM_HPP_
