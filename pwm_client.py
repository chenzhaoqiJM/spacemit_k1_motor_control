import zmq

ctx = zmq.Context()
sock = ctx.socket(zmq.REQ)
sock.connect("ipc:///run/pwm.sock")

print("输入 duty (0.0 ~ 1.0)，输入 q 退出")

while True:
    s = input("duty> ").strip()

    if s.lower() in ("q", "quit", "exit"):
        break

    try:
        duty = float(s)
    except ValueError:
        print("错误，请输入数字")
        continue

    if not 0.0 <= duty <= 1.0:
        print("错误，duty 必须在 0.0 ~ 1.0 之间")
        continue

    # REQ → REP：send 后必须 recv
    sock.send_json({"duty": duty})
    reply = sock.recv_json()

    print("✔ 回复:", reply)
