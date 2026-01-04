#include "encoder_speed_meter.hpp"

#include <cstdio>
#include <iostream>

using namespace std::chrono;

/* ================= EncoderSpeedMeter ================= */

EncoderSpeedMeter::EncoderSpeedMeter(int gpio_line, int chip_index,
                                     double sample_period, double encoder_ppr,
                                     double encoder_edges, double gear_ratio,
                                     double alpha, size_t queue_size)
    : gpio_line_(gpio_line), chip_index_(chip_index),
      sample_period_(sample_period), encoder_ppr_(encoder_ppr),
      encoder_edges_(encoder_edges), gear_ratio_(gear_ratio), alpha_(alpha),
      queue_size_(queue_size) {}

EncoderSpeedMeter::~EncoderSpeedMeter() { stop(); }

bool EncoderSpeedMeter::start() {
  if (!init_gpio())
    return false;

  stop_flag_ = false;
  sampler_thread_ = std::thread(&EncoderSpeedMeter::sampler_loop, this);
  processor_thread_ = std::thread(&EncoderSpeedMeter::processor_loop, this);
  irq_thread_ = std::thread(&EncoderSpeedMeter::irq_loop, this);

  return true;
}

void EncoderSpeedMeter::stop() {
  if (stop_flag_)
    return;

  stop_flag_ = true;
  cv_.notify_all();

  if (irq_thread_.joinable())
    irq_thread_.join();
  if (sampler_thread_.joinable())
    sampler_thread_.join();
  if (processor_thread_.joinable())
    processor_thread_.join();

  if (line_) {
    gpiod_line_release(line_);
    line_ = nullptr;
  }
  if (chip_) {
    gpiod_chip_close(chip_);
    chip_ = nullptr;
  }
}

/* ---------- GPIO ---------- */
bool EncoderSpeedMeter::init_gpio() {
  std::string chip_name = "/dev/gpiochip" + std::to_string(chip_index_);
  chip_ = gpiod_chip_open(chip_name.c_str());
  if (!chip_) {
    std::cerr << "Failed to open " << chip_name << "\n";
    return false;
  }

  line_ = gpiod_chip_get_line(chip_, gpio_line_);
  if (!line_) {
    std::cerr << "Failed to get GPIO line " << gpio_line_ << "\n";
    return false;
  }

  if (gpiod_line_request_both_edges_events(line_, "encoder") < 0) {
    std::cerr << "Failed to request edge events\n";
    return false;
  }

  return true;
}

/* ---------- IRQ 线程 ---------- */
void EncoderSpeedMeter::irq_loop() {
  // 边沿状态机：只有检测到完整的"上升沿->下降沿"序列才计数
  // 这样可以过滤毛刺（同一边沿连续触发不会被重复计数）
  bool last_was_up = false;

  while (!stop_flag_) {
    timespec timeout{1, 0}; // 1s
    int ret = gpiod_line_event_wait(line_, &timeout);
    if (ret <= 0)
      continue;

    gpiod_line_event ev;
    if (gpiod_line_event_read(line_, &ev) != 0)
      continue;

    if (ev.event_type == GPIOD_LINE_EVENT_RISING_EDGE) {
      // 记录看到了上升降沿
      last_was_up = true;

    } else if (ev.event_type == GPIOD_LINE_EVENT_FALLING_EDGE) {

      // 只有在上升沿之后的下降沿才计数
      if (last_was_up) {
        pulse_count_++;
        last_was_up = false; // 重置状态，等待下一个上升沿
      }
      // 如果没有先看到上升沿，可能是毛刺，忽略
    }
  }
}

/* ---------- 采样线程 ---------- */
void EncoderSpeedMeter::sampler_loop() {
  auto last = steady_clock::now();

  while (!stop_flag_) {
    std::this_thread::sleep_for(duration<double>(sample_period_));

    auto now = steady_clock::now();
    double dt = duration<double>(now - last).count();
    last = now;

    int pulse = pulse_count_.exchange(0);

    std::unique_lock<std::mutex> lock(mtx_);
    if (queue_.size() >= queue_size_) {
      queue_.pop();
    }
    queue_.emplace(dt, pulse);
    cv_.notify_one();
  }
}

/* ---------- 计算线程 ---------- */
void EncoderSpeedMeter::processor_loop() {
  while (!stop_flag_) {
    std::unique_lock<std::mutex> lock(mtx_);
    cv_.wait_for(lock, 200ms, [&] { return !queue_.empty() || stop_flag_; });

    if (queue_.empty())
      continue;

    auto [dt, pulse] = queue_.front();
    queue_.pop();
    lock.unlock();

    double encoder_circles = pulse / (encoder_ppr_ * encoder_edges_);
    double motor_circles = encoder_circles / gear_ratio_;
    double speed = (dt > 0) ? motor_circles / dt : 0.0;

    // 一阶低通
    // speed_filt_ = alpha_ * speed_filt_ + (1 - alpha_) * speed;
    speed_filt_ = speed;

    printf("dt: %.3f ms, Pulse: %d, 转/s: %.3f, 转/min: %.3f\n", dt * 1000.0,
           pulse, speed, speed * 60.0);
  }
}
