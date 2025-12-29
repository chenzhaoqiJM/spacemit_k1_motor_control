#!/usr/bin/env python3
# 以 root 启动

import socket
import os
import time
import json

# ================== PWM 配置 ==================
PWMS = {
    1: {
        "chip": "/sys/class/pwm/pwmchip0",
        "index": 0,
    },
    2: {
        "chip": "/sys/class/pwm/pwmchip1",
        "index": 0,
    },
}

PERIOD_NS = 1_000_000  # 1 ms = 1 kHz
SOCK_PATH = "/run/pwm_control_uds.sock"

# ================== 文件工具 ==================
def write(path, value):
    with open(path, "w") as f:
        f.write(str(value))

def read(path):
    with open(path, "r") as f:
        return f.read().strip()

def pwm_base(chip, index):
    return f"{chip}/pwm{index}"

def ensure_pwm_exported(chip, index):
    base = pwm_base(chip, index)
    if not os.path.exists(base):
        write(f"{chip}/export", index)
        for _ in range(20):
            if os.path.exists(base):
                return
            time.sleep(0.05)
        raise RuntimeError(f"{base} not created")

# ================== PWM 初始化 ==================
for pwm_id, cfg in PWMS.items():
    chip = cfg["chip"]
    index = cfg["index"]
    base = pwm_base(chip, index)

    ensure_pwm_exported(chip, index)

    enable_path = f"{base}/enable"
    if read(enable_path) != "0":
        write(enable_path, 0)

    write(f"{base}/period", PERIOD_NS)
    write(f"{base}/duty_cycle", 0)
    write(enable_path, 1)

    cfg["base"] = base  # 缓存路径

print("PWM initialized:", ", ".join(f"pwm_motor_{pid}" for pid in PWMS))

# ================== UNIX Domain Socket ==================
if os.path.exists(SOCK_PATH):
    os.unlink(SOCK_PATH)

srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
srv.bind(SOCK_PATH)
os.chmod(SOCK_PATH, 0o666)   # 允许普通用户访问
srv.listen(1)

print("PWM UDS server listening:", SOCK_PATH)

# ================== 主循环 ==================
while True:
    conn, _ = srv.accept()
    print("client connected")

    while True:
        try:
            data = conn.recv(256)
            if not data:
                # client 正常 close
                break
        except ConnectionResetError:
            # client 异常退出（RST）
            print("client reset connection")
            break
        except Exception as e:
            print("recv error:", e)
            break

        try:
            msg = json.loads(data.decode())

            duty_motor1 = float(msg.get("duty_motor1", 0.0))
            duty_motor2 = float(msg.get("duty_motor2", 0.0))

            # motor1
            duty_motor1 = max(0.0, min(1.0, duty_motor1))
            base_motor1 = PWMS[1]["base"]
            write(f"{base_motor1}/duty_cycle", int(PERIOD_NS * duty_motor1))

            # motor2
            duty_motor2 = max(0.0, min(1.0, duty_motor2))
            base_motor2 = PWMS[2]["base"]
            write(f"{base_motor2}/duty_cycle", int(PERIOD_NS * duty_motor2))

            resp = {"ok": True}

        except Exception as e:
            resp = {"ok": False, "error": str(e)}

        try:
            conn.sendall(json.dumps(resp).encode())
        except BrokenPipeError:
            print("client pipe broken while sending")
            break


    conn.close()
    print("client disconnected")
