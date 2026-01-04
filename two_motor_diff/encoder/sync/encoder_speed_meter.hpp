#ifndef ENCODER_SPEED_METER_HPP
#define ENCODER_SPEED_METER_HPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

namespace gpiod {
class chip;
class line;
struct line_event;
} // namespace gpiod

/**
 * @brief 编码器测速器
 *
 * 通过 GPIO 中断监听编码器脉冲，计算电机转速 (RPS / RPM)
 */
class EncoderSpeedMeter {
public:
  /**
   * @param gpio_offset   GPIO 引脚编号
   * @param chip_path     GPIO 芯片路径 (如 /dev/gpiochip0)
   * @param sample_period 采样周期 (秒)
   * @param encoder_ppr   编码器每圈脉冲数
   * @param gear_ratio    减速比
   * @param queue_size    采样队列大小
   */
  EncoderSpeedMeter(unsigned int gpio_offset,
                    const std::string &chip_path = "/dev/gpiochip0",
                    double sample_period = 0.033, double encoder_ppr = 11.0,
                    double gear_ratio = 56.0, size_t queue_size = 10);

  ~EncoderSpeedMeter();

  /// 启动测速
  void start();

  /// 停止测速
  void stop();

  /// 获取当前转速 (转/秒)
  double get_rps() const;

  /// 获取当前转速 (转/分钟)
  double get_rpm() const;

private:
  /* ---------- GPIO ---------- */
  unsigned int gpio_offset_;
  std::string chip_path_;

  /* ---------- 参数 ---------- */
  double sample_period_;
  double encoder_ppr_;
  double gear_ratio_;
  size_t queue_size_;

  /* ---------- 状态 ---------- */
  mutable std::mutex lock_;
  uint64_t pulse_count_{0};
  bool has_rising_{false};
  std::atomic<double> current_rps_{0.0};

  /* ---------- 线程通信 ---------- */
  std::queue<std::pair<double, uint64_t>> queue_;
  std::atomic<bool> stop_flag_{true};
  std::condition_variable cv_;

  /* ---------- 线程 ---------- */
  std::thread interrupt_thread_;
  std::thread sampler_thread_;
  std::thread processor_thread_;

  /* ---------- 私有方法 ---------- */
  void interrupt_loop();
  void handle_event(const gpiod::line_event &event);
  void sampler_loop();
  void processor_loop();
};

#endif // ENCODER_SPEED_METER_HPP
