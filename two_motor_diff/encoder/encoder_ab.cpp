#include <gpiod.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>

using namespace std::chrono;

static std::atomic<bool> g_running{true};

/* ================= QuadratureEncoder (AB相编码器) ================= */

class QuadratureEncoder {
public:
  QuadratureEncoder(int gpio_a, int gpio_b, int chip_index = 0,
                    double sample_period = 0.033, double encoder_ppr = 11.0,
                    double gear_ratio = 56.0, double alpha = 0.85,
                    size_t queue_size = 10)
      : gpio_a_(gpio_a), gpio_b_(gpio_b), chip_index_(chip_index),
        sample_period_(sample_period), encoder_ppr_(encoder_ppr),
        gear_ratio_(gear_ratio), alpha_(alpha), queue_size_(queue_size) {}

  ~QuadratureEncoder() { stop(); }

  bool start() {
    if (!init_gpio())
      return false;

    stop_flag_ = false;
    sampler_thread_ = std::thread(&QuadratureEncoder::sampler_loop, this);
    processor_thread_ = std::thread(&QuadratureEncoder::processor_loop, this);
    irq_a_thread_ = std::thread(&QuadratureEncoder::irq_a_loop, this);
    irq_b_thread_ = std::thread(&QuadratureEncoder::irq_b_loop, this);

    return true;
  }

  void stop() {
    if (stop_flag_)
      return;

    stop_flag_ = true;
    cv_.notify_all();

    if (irq_a_thread_.joinable())
      irq_a_thread_.join();
    if (irq_b_thread_.joinable())
      irq_b_thread_.join();
    if (sampler_thread_.joinable())
      sampler_thread_.join();
    if (processor_thread_.joinable())
      processor_thread_.join();

    if (line_a_) {
      gpiod_line_release(line_a_);
      line_a_ = nullptr;
    }
    if (line_b_) {
      gpiod_line_release(line_b_);
      line_b_ = nullptr;
    }
    if (chip_) {
      gpiod_chip_close(chip_);
      chip_ = nullptr;
    }
  }

  double speed_rps() const { return speed_filt_; }

  double speed_rpm() const { return speed_filt_ * 60.0; }

  int position() const { return position_; }

private:
  /* ---------- GPIO ---------- */
  bool init_gpio() {
    std::string chip_name = "/dev/gpiochip" + std::to_string(chip_index_);
    chip_ = gpiod_chip_open(chip_name.c_str());
    if (!chip_) {
      std::cerr << "Failed to open " << chip_name << "\n";
      return false;
    }

    // 初始化 A 相
    line_a_ = gpiod_chip_get_line(chip_, gpio_a_);
    if (!line_a_) {
      std::cerr << "Failed to get GPIO line A (" << gpio_a_ << ")\n";
      return false;
    }
    if (gpiod_line_request_both_edges_events(line_a_, "encoder_a") < 0) {
      std::cerr << "Failed to request edge events for A\n";
      return false;
    }

    // 初始化 B 相
    line_b_ = gpiod_chip_get_line(chip_, gpio_b_);
    if (!line_b_) {
      std::cerr << "Failed to get GPIO line B (" << gpio_b_ << ")\n";
      return false;
    }
    if (gpiod_line_request_both_edges_events(line_b_, "encoder_b") < 0) {
      std::cerr << "Failed to request edge events for B\n";
      return false;
    }

    // 读取初始状态
    last_a_ = gpiod_line_get_value(line_a_);
    last_b_ = gpiod_line_get_value(line_b_);

    return true;
  }

  /* ---------- A相 IRQ 线程 ---------- */
  void irq_a_loop() {
    bool last_was_up = false;

    while (!stop_flag_) {
      timespec timeout{1, 0}; // 1s
      int ret = gpiod_line_event_wait(line_a_, &timeout);
      if (ret <= 0)
        continue;

      gpiod_line_event ev;
      if (gpiod_line_event_read(line_a_, &ev) != 0)
        continue;

      if (ev.event_type == GPIOD_LINE_EVENT_RISING_EDGE) {
        // 记录看到了上升沿
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

  /* ---------- B相 IRQ 线程 ---------- */
  void irq_b_loop() {
    bool last_was_up = false;

    while (!stop_flag_) {
      timespec timeout{1, 0}; // 1s
      int ret = gpiod_line_event_wait(line_b_, &timeout);
      if (ret <= 0)
        continue;

      gpiod_line_event ev;
      if (gpiod_line_event_read(line_b_, &ev) != 0)
        continue;

      if (ev.event_type == GPIOD_LINE_EVENT_FALLING_EDGE) {
        // 记录看到了上升沿
        last_was_up = true;
      } else if (ev.event_type == GPIOD_LINE_EVENT_RISING_EDGE) {
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
  void sampler_loop() {
    auto last = steady_clock::now();

    while (!stop_flag_) {
      std::this_thread::sleep_for(duration<double>(sample_period_));

      auto now = steady_clock::now();
      double dt = duration<double>(now - last).count();
      last = now;

      int pulse = pulse_count_.exchange(0);
      int pos = position_.load();

      std::unique_lock<std::mutex> lock(mtx_);
      if (queue_.size() >= queue_size_) {
        queue_.pop();
      }
      queue_.emplace(dt, pulse, pos);
      cv_.notify_one();
    }
  }

  /* ---------- 计算线程 ---------- */
  void processor_loop() {
    while (!stop_flag_) {
      std::unique_lock<std::mutex> lock(mtx_);
      cv_.wait_for(lock, 200ms, [&] { return !queue_.empty() || stop_flag_; });

      if (queue_.empty())
        continue;

      auto [dt, pulse, pos] = queue_.front();
      queue_.pop();
      lock.unlock();

      // 2倍频：每个A/B周期产生2个边沿
      double encoder_circles = pulse / (encoder_ppr_ * 2.0);
      double motor_circles = encoder_circles / gear_ratio_;
      double speed = (dt > 0) ? motor_circles / dt : 0.0;

      // 一阶低通
      // speed_filt_ = alpha_ * speed_filt_ + (1 - alpha_) * speed;
      speed_filt_ = speed;

      printf("dt: %.3f ms, Pulse: %d, Pos: %d, 转/s: %.3f, 转/min: %.3f\n",
             dt * 1000.0, pulse, pos, speed, speed * 60.0);
    }
  }

private:
  /* GPIO */
  int gpio_a_;
  int gpio_b_;
  int chip_index_;
  gpiod_chip *chip_{nullptr};
  gpiod_line *line_a_{nullptr};
  gpiod_line *line_b_{nullptr};

  /* 参数 */
  double sample_period_;
  double encoder_ppr_;
  double gear_ratio_;
  double alpha_;
  size_t queue_size_;

  /* 状态 */
  std::atomic<int> last_a_{0};
  std::atomic<int> last_b_{0};
  std::atomic<int> position_{0};    // 位置计数（可正可负）
  std::atomic<int> pulse_count_{0}; // 脉冲计数（绝对值，用于计算速度）
  std::atomic<double> speed_filt_{0.0};
  std::atomic<bool> stop_flag_{false};

  /* 同步 */
  std::mutex mtx_;
  std::condition_variable cv_;
  std::queue<std::tuple<double, int, int>> queue_; // (dt, pulse, position)

  /* 线程 */
  std::thread irq_a_thread_;
  std::thread irq_b_thread_;
  std::thread sampler_thread_;
  std::thread processor_thread_;
};

/* ================= main ================= */

void signal_handler(int) {
  std::cout << "\n[INFO] Ctrl+C received\n";
  g_running = false;
}

int main() {
  std::signal(SIGINT, signal_handler);

  QuadratureEncoder encoder(72,    // GPIO A相
                            73,    // GPIO B相
                            0,     // gpiochip
                            0.033, // sample_period
                            11.0,  // encoder_ppr
                            56.0,  // gear ratio
                            0.2    // alpha
  );

  if (!encoder.start()) {
    std::cerr << "Failed to start quadrature encoder\n";
    return 1;
  }

  while (g_running) {
    std::this_thread::sleep_for(1s);
  }

  encoder.stop();
  return 0;
}
