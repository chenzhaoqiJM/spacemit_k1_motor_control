# 以 root 启动
import socket
import os
import time
import json

PWMCHIP = "/sys/class/pwm/pwmchip0"
PWM_INDEX = 0
PWM_BASE = f"{PWMCHIP}/pwm{PWM_INDEX}"

SOCK_PATH = "/run/pwm_control_uds.sock"

def write(path, value):
    with open(path, "w") as f:
        f.write(str(value))

def read(path):
    with open(path, "r") as f:
        return f.read().strip()

def ensure_pwm_exported():
    if not os.path.exists(PWM_BASE):
        write(f"{PWMCHIP}/export", PWM_INDEX)
        for _ in range(20):
            if os.path.exists(PWM_BASE):
                break
            time.sleep(0.05)
        else:
            raise RuntimeError(f"{PWM_BASE} not created")

# ================== PWM 初始化 ==================
ensure_pwm_exported()

period = 1_000_000  # ns
enable_path = f"{PWM_BASE}/enable"

if read(enable_path) != "0":
    write(enable_path, 0)

write(f"{PWM_BASE}/period", period)
write(f"{PWM_BASE}/duty_cycle", 0)
write(f"{PWM_BASE}/enable", 1)

# ================== UNIX Domain Socket 服务 ==================
if os.path.exists(SOCK_PATH):
    os.unlink(SOCK_PATH)

srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
srv.bind(SOCK_PATH)
os.chmod(SOCK_PATH, 0o666)   # 允许普通用户访问
srv.listen(1)

print("PWM UDS server listening:", SOCK_PATH)

while True:
    conn, _ = srv.accept()
    print("client connected")

    while True:
        data = conn.recv(256)
        if not data:
            break   # 客户端断开

        try:
            msg = json.loads(data.decode())
            duty = float(msg.get("duty", 0.0))
            duty = max(0.0, min(1.0, duty))

            write(f"{PWM_BASE}/duty_cycle", int(period * duty))
            resp = {"ok": True}
        except Exception as e:
            resp = {"ok": False, "error": str(e)}

        conn.sendall(json.dumps(resp).encode())

    conn.close()
    print("client disconnected")