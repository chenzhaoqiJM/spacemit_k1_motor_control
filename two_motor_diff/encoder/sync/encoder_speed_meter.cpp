#include "encoder_speed_meter.hpp"
#include <gpiod.hpp>
#include <iostream>

EncoderSpeedMeter::EncoderSpeedMeter(unsigned int gpio_offset,
                                     const std::string &chip_path,
                                     double sample_period, double encoder_ppr,
                                     double gear_ratio, size_t queue_size)
    : gpio_offset_(gpio_offset), chip_path_(chip_path),
      sample_period_(sample_period), encoder_ppr_(encoder_ppr),
      gear_ratio_(gear_ratio), queue_size_(queue_size) {}

EncoderSpeedMeter::~EncoderSpeedMeter() { stop(); }

void EncoderSpeedMeter::start() {
  if (!stop_flag_)
    return; // 已在运行

  stop_flag_ = false;
  interrupt_thread_ = std::thread(&EncoderSpeedMeter::interrupt_loop, this);
  sampler_thread_ = std::thread(&EncoderSpeedMeter::sampler_loop, this);
  processor_thread_ = std::thread(&EncoderSpeedMeter::processor_loop, this);
}

void EncoderSpeedMeter::stop() {
  if (stop_flag_)
    return;

  stop_flag_ = true;
  cv_.notify_all();

  if (interrupt_thread_.joinable())
    interrupt_thread_.join();
  if (sampler_thread_.joinable())
    sampler_thread_.join();
  if (processor_thread_.joinable())
    processor_thread_.join();
}

double EncoderSpeedMeter::get_rps() const { return current_rps_.load(); }

double EncoderSpeedMeter::get_rpm() const { return current_rps_.load() * 60.0; }

/* ================== GPIO 中断监听线程 ================== */
void EncoderSpeedMeter::interrupt_loop() {
  gpiod::chip chip(chip_path_);
  gpiod::line line = chip.get_line(gpio_offset_);

  line.request({"encoder", gpiod::line_request::EVENT_BOTH_EDGES,
                gpiod::line_request::FLAG_BIAS_PULL_UP});

  while (!stop_flag_) {
    if (!line.event_wait(std::chrono::milliseconds(100)))
      continue;

    gpiod::line_event event = line.event_read();
    handle_event(event);
  }
}

/* ================== 中断逻辑 ================== */
void EncoderSpeedMeter::handle_event(const gpiod::line_event &event) {
  std::lock_guard<std::mutex> guard(lock_);

  if (event.event_type == gpiod::line_event::RISING_EDGE) {
    has_rising_ = true;
  } else if (event.event_type == gpiod::line_event::FALLING_EDGE) {
    if (has_rising_) {
      pulse_count_++;
      has_rising_ = false;
      cv_.notify_all();
    }
  }
}

/* ================== 采样线程 ================== */
void EncoderSpeedMeter::sampler_loop() {
  auto last = std::chrono::steady_clock::now();

  while (!stop_flag_) {
    std::this_thread::sleep_for(std::chrono::duration<double>(sample_period_));

    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> dt = now - last;
    last = now;

    uint64_t pulse = 0;
    {
      std::unique_lock<std::mutex> lk(lock_);
      // 如果正在数一个脉冲，等它完成
      cv_.wait(lk, [&] { return !has_rising_; });

      pulse = pulse_count_;
      pulse_count_ = 0;
    }

    if (queue_.size() < queue_size_) {
      queue_.emplace(dt.count(), pulse);
    }
  }
}

/* ================== 计算线程 ================== */
void EncoderSpeedMeter::processor_loop() {
  while (!stop_flag_) {
    if (queue_.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }

    auto [dt, pulse] = queue_.front();
    queue_.pop();

    if (dt <= 0.0)
      continue;

    double encoder_turns = static_cast<double>(pulse) / encoder_ppr_;
    double motor_turns = encoder_turns / gear_ratio_;
    double rps = motor_turns / dt;

    current_rps_.store(rps);

    printf("dt: %.3f ms, Pulse: %ld, 转/s: %.3f, 转/min: %.3f\n", dt * 1000.0,
           pulse, rps, rps * 60.0);
  }
}
