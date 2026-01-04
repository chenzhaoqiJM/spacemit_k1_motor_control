#ifndef PID_ENCODER_HPP
#define PID_ENCODER_HPP

#include <gpiod.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

/* ================= EncoderSpeedMeter ================= */

class EncoderSpeedMeter {
public:
  EncoderSpeedMeter(int gpio_line, int chip_index = 0,
                    double sample_period = 0.033, double encoder_ppr = 11.0,
                    double encoder_edges = 1.0, double gear_ratio = 56.0,
                    double alpha = 0.85, size_t queue_size = 10);

  ~EncoderSpeedMeter();

  bool start();
  void stop();

  double speed_rps() const { return speed_filt_; }
  double speed_rpm() const { return speed_filt_ * 60.0; }

private:
  /* ---------- GPIO ---------- */
  bool init_gpio();

  /* ---------- 线程函数 ---------- */
  void irq_loop();
  void sampler_loop();
  void processor_loop();

private:
  /* GPIO */
  int gpio_line_;
  int chip_index_;
  gpiod_chip *chip_{nullptr};
  gpiod_line *line_{nullptr};

  /* 参数 */
  double sample_period_;
  double encoder_ppr_;
  double encoder_edges_;
  double gear_ratio_;
  double alpha_;
  size_t queue_size_;

  /* 状态 */
  std::atomic<int> pulse_count_{0};
  std::atomic<double> speed_filt_{0.0};
  std::atomic<bool> stop_flag_{false};

  /* 同步 */
  std::mutex mtx_;
  std::condition_variable cv_;
  std::queue<std::pair<double, int>> queue_;

  /* 线程 */
  std::thread irq_thread_;
  std::thread sampler_thread_;
  std::thread processor_thread_;
};

#endif // PID_ENCODER_HPP
