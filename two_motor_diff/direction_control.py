from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device
Device.pin_factory = LGPIOFactory(chip=0)

from gpiozero import LED

class MotorDirectionCtrl:

    def __init__(self):
        # motor1
        self.motor1_IN1 = LED(49)
        self.motor1_IN2 = LED(50)

        # motor2
        self.motor2_IN1 = LED(91)
        self.motor2_IN2 = LED(92)

    # 0停止、1前进、2后退
    def motor1_direction(self, direction: int):
        if direction == 0:
            self.motor1_IN1.off(); self.motor1_IN2.off()
        elif direction == 1:
            self.motor1_IN1.on(); self.motor1_IN2.off()
        elif direction == 2:
            self.motor1_IN1.off(); self.motor1_IN2.on()

    def motor2_direction(self, direction: int):
        if direction == 0:
            self.motor2_IN1.off(); self.motor2_IN2.off()
        elif direction == 1:
            self.motor2_IN1.on(); self.motor2_IN2.off()
        elif direction == 2:
            self.motor2_IN1.off(); self.motor2_IN2.on()

    def close_all(self):
        self.motor1_IN1.close()
        self.motor1_IN2.close()
        self.motor2_IN1.close()
        self.motor2_IN2.close()
