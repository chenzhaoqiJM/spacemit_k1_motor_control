# PWM UNIX Domain Socket Server (C++)

这是一个 **基于 sysfs PWM + UNIX Domain Socket 的 C++ PWM 控制服务**，  
用于在 Linux 嵌入式系统中以 **root 权限**运行，普通用户通过 socket 控制 PWM 占空比。

---

## 功能特性

- sysfs PWM 自动 export / 初始化
- 支持多个 PWM 通道
- UNIX Domain Socket (`/run/pwm_control_uds.sock`)
- JSON 协议控制
- 稳定处理 client 断连 / 异常退出
- 易于 systemd / ROS2 集成

---

## 依赖

### 系统依赖
```bash
sudo apt update
sudo apt install -y build-essential nlohmann-json3-dev
```

## 编译

```
g++ -std=c++17 pwm_server_uds.cpp -o pwm_server_uds
```

或开启优化：

```
g++ -O2 -std=c++17 pwm_server_uds.cpp -o pwm_server_uds
```

## 运行

必须 root：

```
sudo ./pwm_server_uds
```

启动后监听：

```
/run/pwm_control_uds.sock
```