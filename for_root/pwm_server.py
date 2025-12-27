# 以 root 启动
import zmq
import os
import time

PWMCHIP = "/sys/class/pwm/pwmchip0"
PWM_INDEX = 0
PWM_BASE = f"{PWMCHIP}/pwm{PWM_INDEX}"

def write(path, value):
    with open(path, "w") as f:
        f.write(str(value))

def read(path):
    with open(path, "r") as f:
        return f.read().strip()

def ensure_pwm_exported():
    if not os.path.exists(PWM_BASE):
        # export
        write(f"{PWMCHIP}/export", PWM_INDEX)

        # 等待内核创建 pwmX 目录（很重要）
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

# 只有在当前值不是 0 时才写 0
if read(enable_path) != "0":
    write(enable_path, 0)
write(f"{PWM_BASE}/period", period)
write(f"{PWM_BASE}/duty_cycle", 0)
write(f"{PWM_BASE}/enable", 1)

# ================== ZMQ 服务 ==================
ctx = zmq.Context()
sock = ctx.socket(zmq.REP)
sock.bind("ipc:///run/pwm.sock")
os.chmod("/run/pwm.sock", 0o666) # 赋予权限，非常重要，否则普通用户无法使用服务

while True:
    msg = sock.recv_json()
    duty = msg.get("duty", 0.0)

    # 参数校验（非常重要）
    duty = max(0.0, min(1.0, float(duty)))

    write(f"{PWM_BASE}/duty_cycle", int(period * duty))

    sock.send_json({"ok": True})
