from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device
Device.pin_factory = LGPIOFactory(chip=0)

from gpiozero import DigitalInputDevice
import time
from signal import pause
import threading

# 假设方波接在 GPIO17
pulse = DigitalInputDevice(73)

count = 0
lock = threading.Lock()  # 避免多线程同时访问 count

def pulse_detected():
    global count
    with lock:
        count += 1

pulse.when_activated = pulse_detected
pulse.when_deactivated = pulse_detected
def print_count_periodically():
    global count
    while True:
        time.sleep(0.1)  # 0.1 秒周期
        with lock:
            print("Pulse count in last 0.1s:", count)
            count = 0

# 启动打印线程
threading.Thread(target=print_count_periodically, daemon=True).start()

pause()  # 保持主线程运行