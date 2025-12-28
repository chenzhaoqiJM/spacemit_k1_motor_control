import socket
import json

SOCK_PATH = "/run/pwm_control_uds.sock"

sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
sock.connect(SOCK_PATH)


print("输入 duty_motor1;duty_motor2 (0.0 ~ 1.0;0.0 ~ 1.0)，输入 q 退出")

while True:
    s = input("duty_motor1;duty_motor2> ").strip()

    if s.lower() in ("q", "quit", "exit"):
        break

    try:
        index_duty = str(s)
        arr_ = index_duty.split(';')
        if len(arr_) != 2:
            print("输入格式错误, 请重新输入...!")
            continue
        
        duty_motor1 = float(arr_[0])
        duty_motor2 = float(arr_[1])
    except ValueError:
        print("错误，字符串内格式错误")
        continue

    if not 0.0 <= duty_motor1 <= 1.0:
        print("错误, duty_motor1 必须在 0.0 ~ 1.0 之间")
        continue

    if not 0.0 <= duty_motor2 <= 1.0:
        print("错误, duty_motor2 必须在 0.0 ~ 1.0 之间")
        continue

    # REQ → REP：send 后必须 recv
    sock.sendall(json.dumps({"duty_motor1":duty_motor1,"duty_motor2": duty_motor2}).encode())
    resp = sock.recv(256)

    print("✔ 回复:", json.loads(resp.decode()))

sock.close()
