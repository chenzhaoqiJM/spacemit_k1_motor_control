#include "encoder_speed_meter.hpp"

#include <csignal>
#include <iostream>
#include <thread>

/* ================== main ================== */

static EncoderSpeedMeter *g_meter = nullptr;

void sigint_handler(int) {
  if (g_meter)
    g_meter->stop();
  std::exit(0);
}

int main() {
  EncoderSpeedMeter meter(73, "/dev/gpiochip0", 0.033, 11.0, 56.0);

  g_meter = &meter;
  std::signal(SIGINT, sigint_handler);

  std::cout << "开始测量..." << std::endl;
  meter.start();

  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // 示例: 外部获取转速
    // double rps = meter.get_rps();
    // double rpm = meter.get_rpm();
  }
}
