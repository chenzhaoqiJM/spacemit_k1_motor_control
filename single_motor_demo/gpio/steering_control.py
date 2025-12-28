from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device
Device.pin_factory = LGPIOFactory(chip=0)

from gpiozero import LED
import time


led1 = LED(49)
led2 = LED(50)

try:
    while True:
        led1.on()
        led2.off()
        time.sleep(10)  # 等待 10 秒
        led1.off()
        led2.on()
        time.sleep(10)

except KeyboardInterrupt:
    # 捕捉 Ctrl+C 并退出
    print("Exiting")

led1.close()
led2.close()