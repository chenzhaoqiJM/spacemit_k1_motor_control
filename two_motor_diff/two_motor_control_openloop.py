import socket
import json
from direction_control import MotorDirectionCtrl

# 初始化pwm控制客户端
SOCK_PATH = "/run/pwm_control_uds.sock"
sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
sock.connect(SOCK_PATH)

# 初始化电机方向控制
dir_controller = MotorDirectionCtrl()


print("输入 direction1,duty_motor1;direction2,duty_motor2 0/1/2,0.0 ~ 1.0;0,1,2/0.0 ~ 1.0 ，输入 q 退出")

while True:
    s = input("direction1,duty_motor1;direction2,duty_motor2> ").strip()

    if s.lower() in ("q", "quit", "exit"):
        break

    try:
        all_context = str(s)
        all_contest_arr = all_context.split(';')
        if len(all_contest_arr) != 2:
            print("输入格式错误, 请重新输入...!")
            continue
        
        motor1_arr = all_contest_arr[0].split(',')
        motor1_direction, duty_motor1 = int(motor1_arr[0]), float(motor1_arr[1])
        
        motor2_arr = all_contest_arr[1].split(',')
        motor2_direction, duty_motor2 = int(motor2_arr[0]), float(motor2_arr[1])
    except ValueError:
        print("错误，字符串内格式错误")
        continue

    if not 0.0 <= duty_motor1 <= 1.0:
        print("错误, duty_motor1 必须在 0.0 ~ 1.0 之间")
        continue

    if not 0.0 <= duty_motor2 <= 1.0:
        print("错误, duty_motor2 必须在 0.0 ~ 1.0 之间")
        continue

    # 设置方向
    dir_controller.motor1_direction(motor1_direction)
    dir_controller.motor2_direction(motor2_direction)


    # 设置pwm
    sock.sendall(json.dumps({"duty_motor1":duty_motor1,"duty_motor2": duty_motor2}).encode())
    resp = sock.recv(256)

    print("✔ 回复:", json.loads(resp.decode()))

sock.close()
