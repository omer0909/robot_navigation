#include <math.h>

#include <fstream>
#include <iostream>

#include <gpiod.h>
#include <string>
#include <thread>
#include <atomic>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdio>
#include <cmath>


#include "ros2_control_demo_example_2/diffbot_system.hpp"

namespace ros2_control_demo_example_2 {

PWM::PWM(Channel channel_) : channel(channel_) {
  writeToFile(chip + "/export", channel == Channel::Pwm0 ? "0" : "1");
  path = chip + "/pwm" + (channel == Channel::Pwm0 ? "0" : "1");
  writeToFile(path + "/period", "100000");
  set_duty(0);
  writeToFile(path + "/enable", "1");
}

void PWM::set_duty(double duty) {
  duty = std::max(0.0, std::min(1.0, duty));
  writeToFile(path + "/duty_cycle", std::to_string((int)(100000 * duty)));
}

PWM::~PWM() {
  set_duty(0);
  writeToFile(path + "/enable", "0");
  writeToFile(chip + "/unexport", (channel == Channel::Pwm0 ? "0" : "1"));
}

void PWM::writeToFile(const std::string& path, const std::string& value) {
  std::ofstream file(path);
  if (!file.is_open()) {
    std::cerr << "Can not write: " << path << std::endl;
    exit(1);
  }
  file << value;
  file.close();
}


Motor::Motor(){
  chip = gpiod_chip_open_by_name("gpiochip0");
  if (!chip) {
    std::cout << "GPIO chip could not be opened!" << std::endl;
    exit(1);
  }

  motor_dir_l = gpiod_chip_get_line(chip, 19);
  motor_dir_r = gpiod_chip_get_line(chip, 16);

  gpiod_line_request_output(motor_dir_l, "left_motor_dir", 0);
  gpiod_line_request_output(motor_dir_r, "right_motor_dir", 0);

  std::cout << "GPIO chip opened successfully." << std::endl;

  encoder_listener_thread_l = std::thread(&Motor::encoder_listener_l, this);
  encoder_listener_thread_r = std::thread(&Motor::encoder_listener_r, this);
}

void Motor::encoder_listener_l(){
  int fd = open("/dev/i2c-0", O_RDWR);
    if (fd < 0 || ioctl(fd, I2C_SLAVE, AS5600_ADDR) < 0) {
        printf("Hata: I2C açılamadı veya sensör bulunamadı.\n");
        return;
    }

    auto last_time = std::chrono::steady_clock::now();

    int64_t last_raw_val = 0;
    bool first_run = true;

    uint8_t buffer[2];
    uint8_t reg_addr = AS5600_REG_RAW_ANGLE;
    while (active) {
        write(fd, &reg_addr, 1);        
        if (read(fd, buffer, 2) == 2) {
            int raw_val = (buffer[0] << 8) | buffer[1];
            if (first_run){
              last_raw_val = raw_val;
              last_time = std::chrono::steady_clock::now();
              first_run = false;
              continue;
            }

            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<double> dt_duration = now - last_time;
            double dt = dt_duration.count();
            last_time = now;

            int diff = raw_val - last_raw_val;
            if (diff > AS5600_RES / 2){
              diff -= AS5600_RES;
            } else if (diff < -AS5600_RES / 2){
              diff += AS5600_RES;
            }
            pos_l += diff;
            last_raw_val = raw_val;

            double vel = diff / dt;

            set_duty_l((-vel * DAMPING) + (target_pos_l - pos_l) * STIFFNESS);
            double gear_ratio = 60.0 / 16.0;
            target_pos_l += (target_vel_l / (2.0 * M_PI)) * gear_ratio * AS5600_RES * dt;
          }
    }
    close(fd);
}

void Motor::encoder_listener_r(){
    int fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0 || ioctl(fd, I2C_SLAVE, AS5600_ADDR) < 0) {
        printf("Hata: I2C açılamadı veya sensör bulunamadı.\n");
        return;
    }

    auto last_time = std::chrono::steady_clock::now();

    int64_t last_raw_val = 0;
    bool first_run = true;

    uint8_t buffer[2];
    uint8_t reg_addr = AS5600_REG_RAW_ANGLE;
    while (active) {
        write(fd, &reg_addr, 1);        
        if (read(fd, buffer, 2) == 2) {
            int raw_val = (buffer[0] << 8) | buffer[1];
            if (first_run){
              last_raw_val = raw_val;
              last_time = std::chrono::steady_clock::now();
              first_run = false;
              continue;
            }

            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<double> dt_duration = now - last_time;
            double dt = dt_duration.count();
            last_time = now;

            int diff = raw_val - last_raw_val;
            if (diff > AS5600_RES / 2){
              diff -= AS5600_RES;
            } else if (diff < -AS5600_RES / 2){
              diff += AS5600_RES;
            }
            pos_r += diff;
            last_raw_val = raw_val;

            double vel = diff / dt;

            set_duty_r((-vel * DAMPING) + (target_pos_r - pos_r) * STIFFNESS);
            double gear_ratio = 60.0 / 16.0;
            target_pos_r += (target_vel_r / (2.0 * M_PI)) * gear_ratio * AS5600_RES * dt;
          }
    }
    close(fd);
}

Motor::~Motor(){
  active = false;
  encoder_listener_thread_l.join();
  encoder_listener_thread_r.join();

  gpiod_line_set_value(motor_dir_l, 0);
  gpiod_line_set_value(motor_dir_r, 0);
  gpiod_line_release(motor_dir_l);
  gpiod_line_release(motor_dir_r);

  gpiod_chip_close(chip);
}


void Motor::set_duty_l(double duty) {
  duty = std::min(DUTY_MAX, std::max(-DUTY_MAX, duty));
  if (duty < 0) {
    gpiod_line_set_value(motor_dir_l, 1);
    motor_pwm_l.set_duty(1.0 + duty);
  } else {
    gpiod_line_set_value(motor_dir_l, 0);
    motor_pwm_l.set_duty(duty);
  }
}

void Motor::set_duty_r(double duty) {
  duty = std::min(DUTY_MAX, std::max(-DUTY_MAX, duty));
  if (duty < 0) {
    gpiod_line_set_value(motor_dir_r, 1);
    motor_pwm_r.set_duty(1.0 + duty);
  } else {
    gpiod_line_set_value(motor_dir_r, 0);
    motor_pwm_r.set_duty(duty);
  }
}

void Motor::set_vel_l(double vel) {
  target_vel_l = -vel;
}

void Motor::set_vel_r(double vel) {
  target_vel_r = vel;
}

double Motor::get_pos_l() {
  double gear_ratio = 16.0 / 60.0;
  return -(pos_l / (double)AS5600_RES) * gear_ratio * (2.0 * M_PI);
}

double Motor::get_pos_r() {
  double gear_ratio = 16.0 / 60.0;
  return (pos_r / (double)AS5600_RES) * gear_ratio * (2.0 * M_PI);
}

}  // namespace ros2_control_demo_example_2
