from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device, DigitalInputDevice
import time
import threading
import signal
import sys
import queue


class EncoderSpeedMeter:
    def __init__(
        self,
        gpio_pin: int,
        chip: int = 0,
        sample_period: float = 0.033,
        encoder_ppr: float = 11.0,
        encoder_edges: float = 1.0,
        gear_ratio: float = 56.0,
        alpha: float = 0.85,
        queue_size: int = 10,
    ):
        # 参数
        self.sample_period = sample_period
        self.dt = self.sample_period # 真实测量值
        self.encoder_ppr = encoder_ppr
        self.encoder_edges = encoder_edges
        self.gear_ratio = gear_ratio
        self.alpha = alpha

        # 状态
        self._count = 0
        self._speed_filt = 0.0

        # 同步
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        # 采样数据队列 (dt, pulse)
        self._queue = queue.Queue(maxsize=queue_size)

        # GPIO 初始化
        Device.pin_factory = LGPIOFactory(chip=chip)
        self._pulse = DigitalInputDevice(gpio_pin)

        # 注册中断
        self._pulse.when_activated = self._pulse_detected

        # 线程
        self._sampler = threading.Thread(
            target=self._sampler_loop, daemon=True
        )
        self._processor = threading.Thread(
            target=self._processor_loop, daemon=True
        )

    # ---------------- GPIO 回调 ----------------
    def _pulse_detected(self):
        with self._lock:
            self._count += 1

    # ---------------- 采样线程 ----------------
    def _sampler_loop(self):
        last_time = time.monotonic()

        while not self._stop_event.is_set():
            time.sleep(self.sample_period - 0.0002)

            now = time.monotonic()
            self.dt = now - last_time
            last_time = now

            with self._lock:
                pulse = self._count
                self._count = 0

            if self.dt > 0:
                try:
                    self._queue.put_nowait((self.dt, pulse))
                except queue.Full:
                    # 丢弃最旧或直接丢本次都可以
                    pass

    # ---------------- 计算线程 ----------------
    def _processor_loop(self):
        while not self._stop_event.is_set():
            try:
                dt, pulse = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue

            encoder_circles = pulse / (
                self.encoder_ppr * self.encoder_edges
            )
            motor_circles = encoder_circles / self.gear_ratio
            speed = motor_circles / dt  # rps

            # 一阶低通滤波
            # self._speed_filt = (self.alpha * self._speed_filt + (1 - self.alpha) * speed)
            self._speed_filt = speed

            print(
                f"dt: {dt*1000:.2f} ms, "
                f"Pulse: {pulse}, "
                f"转/s: {self._speed_filt:.3f}, "
                f"转/min: {self._speed_filt * 60:.2f}"
            )

    # ---------------- 对外接口 ----------------
    def start(self):
        self._sampler.start()
        self._processor.start()

    def stop(self):
        self._stop_event.set()
        self._sampler.join()
        self._processor.join()

        self._pulse.when_activated = None
        self._pulse.close()

    def get_speed_rps(self) -> float:
        return self._speed_filt

    def get_speed_rpm(self) -> float:
        return self._speed_filt * 60.0


def main():
    meter = EncoderSpeedMeter(
        gpio_pin=73,
        alpha=0.2,
    )

    def signal_handler(sig, frame):
        print("\n[INFO] Ctrl+C received, exiting...")
        meter.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    meter.start()

    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()