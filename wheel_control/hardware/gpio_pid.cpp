#include <fstream>
#include <iostream>

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

void GpiodPidController::set_vel_r(double vel) {
  pos_r += vel;
}

void GpiodPidController::set_vel_l(double vel) {
  pos_l += vel;
}

int64_t GpiodPidController::get_pos_r() {
  return pos_r;
}

int64_t GpiodPidController::get_pos_l() {
  return pos_l;
}

GpiodPidController::GpiodPidController() {
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

  encoder_listener_thread = std::thread(&GpiodPidController::encoder_listener, this);
  pid_controller_thread = std::thread(&GpiodPidController::pid_controller, this);
}

GpiodPidController::~GpiodPidController() {
  active = false;
  pid_controller_thread.join();
  encoder_listener_thread.join();

  gpiod_line_set_value(motor_dir_l, 0);
  gpiod_line_set_value(motor_dir_r, 0);
  gpiod_line_release(motor_dir_l);
  gpiod_line_release(motor_dir_r);

  gpiod_chip_close(chip);
}

void GpiodPidController::encoder_listener() {
  const int LEFT_PIN1 = 4;
  const int LEFT_PIN2 = 17;
  const int RIGHT_PIN1 = 27;
  const int RIGHT_PIN2 = 22;

  std::vector<int> pin_offsets = {LEFT_PIN1, LEFT_PIN2, RIGHT_PIN1, RIGHT_PIN2};

  bool left_sensor_1 = true;
  bool left_sensor_2 = true;
  bool right_sensor_1 = true;
  bool right_sensor_2 = true;

  std::vector<gpiod_line*> lines;
  for (int offset : pin_offsets) {
    gpiod_line* line = gpiod_chip_get_line(chip, offset);
    if (!line) {
      std::cout << "Pin " << offset << " için GPIO hattı alınamadı." << std::endl;
      gpiod_chip_close(chip);
      exit(1);
    }
    if (gpiod_line_request_both_edges_events(line, "gpiod-poll-example") < 0) {
      std::cout << "Pin " << offset << " için olay isteği yapılamadı." << std::endl;
      gpiod_chip_close(chip);
      exit(1);
    }
    lines.push_back(line);
  }

  while (active) {
    struct gpiod_line_event event;
    constexpr int TIMEOUT_MS = 1000;
    struct timeval timeout = {TIMEOUT_MS / 1000, (TIMEOUT_MS % 1000) * 1000};
    fd_set fds;

    FD_ZERO(&fds);
    int max_fd = -1;
    for (auto& line : lines) {
      int fd = gpiod_line_event_get_fd(line);
      FD_SET(fd, &fds);
      if (fd > max_fd) max_fd = fd;
    }

    int ret = select(max_fd + 1, &fds, nullptr, nullptr, &timeout);
    if (ret < 0) {
      std::cerr << "select() çağrısı başarısız oldu." << std::endl;
      exit(1);
    } else if (ret == 0) {
      continue;
    }

    for (size_t i = 0; i < lines.size(); ++i) {
      int fd = gpiod_line_event_get_fd(lines[i]);
      if (FD_ISSET(fd, &fds) && gpiod_line_event_read(lines[i], &event) == 0 && (event.event_type == GPIOD_LINE_EVENT_RISING_EDGE || event.event_type == GPIOD_LINE_EVENT_FALLING_EDGE)) {
        bool detected = event.event_type == GPIOD_LINE_EVENT_RISING_EDGE;
        int pin = pin_offsets[i];

        if (pin == LEFT_PIN1 || pin == LEFT_PIN2) {
          if (pin == LEFT_PIN1) {
            left_sensor_1 = detected;
            pos_l += (left_sensor_1 == left_sensor_2) ? -1 : 1;
          } else {
            left_sensor_2 = detected;
            pos_l += (left_sensor_1 == left_sensor_2) ? 1 : -1;
          }
        } else {
          if (pin == RIGHT_PIN1) {
            right_sensor_1 = detected;
            pos_r += (right_sensor_1 == right_sensor_2) ? -1 : 1;
          } else {
            right_sensor_2 = detected;
            pos_r += (right_sensor_1 == right_sensor_2) ? 1 : -1;
          }
        }
      }
    }
  }

  for (auto& line : lines) {
    gpiod_line_release(line);
  }
}

void GpiodPidController::pid_controller() {
  while (active) {
    std::cout << "left_angle: " << pos_l << std::endl;
    std::cout << "right_angle: " << pos_r << std::endl;
    set_duty_l(0.2);
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}

void GpiodPidController::set_duty_l(double duty) {
  if (duty < 0) {
    gpiod_line_set_value(motor_dir_l, 1);
    motor_pwm_l.set_duty(1.0 + duty);
  } else {
    gpiod_line_set_value(motor_dir_l, 0);
    motor_pwm_l.set_duty(duty);
  }
}

void GpiodPidController::set_duty_r(double duty) {
  if (duty < 0) {
    gpiod_line_set_value(motor_dir_r, 1);
    motor_pwm_r.set_duty(1.0 + duty);
  } else {
    gpiod_line_set_value(motor_dir_r, 0);
    motor_pwm_r.set_duty(duty);
  }
}

}  // namespace ros2_control_demo_example_2

// #include <chrono>
// #include <iostream>

// class PIDController {
//  private:
//   double kp;  // Proportional gain
//   double ki;  // Integral gain
//   double kd;  // Derivative gain

//   double prev_error;  // Önceki hata
//   double integral;    // Integral terimi
//   double dt;          // Örnekleme zamanı (saniye)
//   std::chrono::steady_clock::time_point last_time;

//  public:
//   // Yapıcı
//   PIDController(double kp, double ki, double kd, double dt)
//       : kp(kp), ki(ki), kd(kd), dt(dt), prev_error(0.0), integral(0.0) {
//     last_time = std::chrono::steady_clock::now();
//   }

//   // PID kontrol çıkışı hesaplama
//   double calculate(double setpoint, double measured_value) {
//     double error = setpoint - measured_value;

//     // Zaman farkını hesapla
//     auto now = std::chrono::steady_clock::now();
//     double elapsed_time = std::chrono::duration<double>(now - last_time).count();
//     last_time = now;

//     // Integral ve türev hesaplama
//     integral += error * elapsed_time;
//     double derivative = (error - prev_error) / elapsed_time;

//     // PID kontrolör formülü
//     double output = kp * error + ki * integral + kd * derivative;

//     // Önceki hatayı güncelle
//     prev_error = error;

//     return output;
//   }
// };

// int main() {
//   PIDController pid(1.0, 0.1, 0.05, 0.01);  // Kp, Ki, Kd ve örnekleme zamanı
//   double setpoint = 100.0;                  // İstenen hedef değer
//   double measured_value = 0.0;              // Başlangıçta ölçülen değer

//   for (int i = 0; i < 100; ++i) {
//     // PID kontrol çıkışını hesapla
//     double control_output = pid.calculate(setpoint, measured_value);

//     // Sistemi kontrol et (örnek olarak sadece çıktı ekleniyor)
//     measured_value += control_output * 0.1;  // Sistemin tepkisini simüle et

//     // Çıkışı yazdır
//     std::cout << "Iteration: " << i
//               << " Setpoint: " << setpoint
//               << " Measured: " << measured_value
//               << " Control Output: " << control_output << std::endl;
//   }

//   return 0;
// }

// #include <gpiod.h>
// #include <unistd.h>  // For close()

// #include <iostream>
// #include <vector>

// #define GPIO_CHIP_NAME "gpiochip0"  // GPIO çipi adı
// #define TIMEOUT_MS 5000             // 5 saniye zaman aşımı

// void handle_event(const gpiod_line_event& event, int line_offset) {
//   // Olayın türüne göre işlem yap
//   if (event.event_type == GPIOD_LINE_EVENT_RISING_EDGE) {
//     std::cout << "Pin " << line_offset << " yükselen kenar algılandı." << std::endl;
//   } else if (event.event_type == GPIOD_LINE_EVENT_FALLING_EDGE) {
//     std::cout << "Pin " << line_offset << " düşen kenar algılandı." << std::endl;
//   }
// }

// int main() {
//   // GPIO pinlerini izlemek için liste
//   std::vector<int> pin_offsets = {4, 17, 27, 22};  // İzlenecek pin numaraları

//   // GPIO çipini aç
//   gpiod_chip* chip = gpiod_chip_open_by_name(GPIO_CHIP_NAME);
//   if (!chip) {
//     std::cerr << "GPIO çipi açılamadı: " << GPIO_CHIP_NAME << std::endl;
//     return 1;
//   }

//   // GPIO hatlarını yapılandır
//   std::vector<gpiod_line*> lines;
//   for (int offset : pin_offsets) {
//     gpiod_line* line = gpiod_chip_get_line(chip, offset);
//     if (!line) {
//       std::cerr << "Pin " << offset << " için GPIO hattı alınamadı." << std::endl;
//       gpiod_chip_close(chip);
//       return 1;
//     }
//     if (gpiod_line_request_both_edges_events(line, "gpiod-poll-example") < 0) {
//       std::cerr << "Pin " << offset << " için olay isteği yapılamadı." << std::endl;
//       gpiod_chip_close(chip);
//       return 1;
//     }
//     lines.push_back(line);
//   }

//   std::cout << "GPIO pinleri izleniyor... (Çıkış için Ctrl+C)" << std::endl;

//   // Olay döngüsü
//   while (true) {
//     struct gpiod_line_event event;
//     struct timeval timeout = {TIMEOUT_MS / 1000, (TIMEOUT_MS % 1000) * 1000};
//     fd_set fds;

//     // File descriptor setini oluştur
//     FD_ZERO(&fds);
//     int max_fd = -1;
//     for (auto& line : lines) {
//       int fd = gpiod_line_event_get_fd(line);
//       FD_SET(fd, &fds);
//       if (fd > max_fd) max_fd = fd;
//     }

//     // poll işlemi
//     int ret = select(max_fd + 1, &fds, nullptr, nullptr, &timeout);
//     if (ret < 0) {
//       std::cerr << "select() çağrısı başarısız oldu." << std::endl;
//       break;
//     } else if (ret == 0) {
//       // Zaman aşımı
//       std::cout << "Zaman aşımı: hiçbir pin değişmedi." << std::endl;
//       continue;
//     }

//     // Olay kontrolü
//     for (size_t i = 0; i < lines.size(); ++i) {
//       int fd = gpiod_line_event_get_fd(lines[i]);
//       if (FD_ISSET(fd, &fds)) {
//         if (gpiod_line_event_read(lines[i], &event) == 0) {
//           handle_event(event, pin_offsets[i]);
//         } else {
//           std::cerr << "Olay okunamadı: Pin " << pin_offsets[i] << std::endl;
//         }
//       }
//     }
//   }

//   // Kaynakları temizle
//   for (auto& line : lines) {
//     gpiod_line_release(line);
//   }
//   gpiod_chip_close(chip);

//   return 0;
// }