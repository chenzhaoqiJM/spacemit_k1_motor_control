
# 引脚使用约定

## motor1
pwm: GPIO34
正反转 IN1、IN2: GPIO49、GPIO50


## motor2
pwm: GPIO35
正反转：IN1、IN2: GPIO91、GPIO92


## 安装依赖

```
sudo apt update
sudo apt install python3-gpiozero libgpiod-dev liblgpio-dev
```


## pwm 服务

pwm_server_uds服务需要使用 sudo 运行


## client 里面是调试程序

