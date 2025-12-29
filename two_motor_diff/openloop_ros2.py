#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import time
import math
import serial
import threading

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

import socket
import json
from direction_control import MotorDirectionCtrl

# ================== 底盘参数 ==================
WHEEL_DIAMETER = 0.067    # m
WHEEL_BASE = 0.33         # m

# 电机模型
def motor_model(speed : float):
    k_12 = 0.2781 
    b_12 = 0.0233
    pwm_duty = k_12 * speed + b_12
    return pwm_duty

# 控制类
class CmdVelToMotor(Node):

    def __init__(self):
        super().__init__('cmdvel_to_serial')

        # ---------------- 参数 ----------------
        self.declare_parameter('send_hz', 40.0)
        self.declare_parameter('cmd_vel_timeout', 0.4)

        # ===== odom 参数（新增）=====
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.send_hz = self.get_parameter('send_hz').value
        self.timeout = self.get_parameter('cmd_vel_timeout').value

        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # 初始化pwm控制客户端
        # pwm4_motor1: GPIO34、pwm5_motor2: GPIO35
        SOCK_PATH = "/run/pwm_control_uds.sock"
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.connect(SOCK_PATH)

        # 初始化电机方向控制
        self.dir_controller = MotorDirectionCtrl() # motor1:49\50 motor2:91\92

        # ---------------- 状态 ----------------
        self.last_cmd_time = time.time()
        self.cur_v = 0.0
        self.cur_w = 0.0

        self.lock = threading.Lock()
        self.running = True

        # ===== odom 状态（新增）=====
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_odom_time = time.monotonic()

        # 轮速缓存（来自轮子编码器）
        self.v_l = 0.0
        self.v_r = 0.0

        # ---------------- ROS ----------------
        self.create_subscription(Twist, 'cmd_vel', self.cmdvel_cb, 10)

        self.odom_pub = self.create_publisher(
            Odometry, self.odom_topic, 10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(
            1.0 / self.send_hz,
            self.send_timer_cb
        )

        # ===== odom 定时器（新增）=====
        self.odom_timer = self.create_timer(
            1.0 / 50.0,
            self.odom_timer_cb
        )

        self.get_logger().info("cmd_vel → control + odom node started")

    # =====================================================
    # cmd_vel 回调
    # =====================================================
    def cmdvel_cb(self, msg: Twist):
        with self.lock:
            self.cur_v = msg.linear.x
            self.cur_w = msg.angular.z
            self.last_cmd_time = time.time()

    # =====================================================
    # 定时器：发送
    # =====================================================
    def send_timer_cb(self):
        now = time.time()

        with self.lock:
            if now - self.last_cmd_time > self.timeout:
                v = 0.0
                w = 0.0
            else:
                v = self.cur_v
                w = self.cur_w

        self.send_cmd(v, w)

    # =====================================================
    # 发送速度到电机
    # =====================================================
    def send_cmd(self, v, w):
        v_l = v - w * WHEEL_BASE / 2.0
        v_r = v + w * WHEEL_BASE / 2.0

        dir_l, spd_l = self.cmd_to_wheel_speed(v_l)
        dir_r, spd_r = self.cmd_to_wheel_speed(v_r)

        cmd = f"{dir_l},{spd_l:.2f};{dir_r},{spd_r:.2f}\n"
        
        # 设置方向
        self.dir_controller.motor1_direction(dir_l)
        self.dir_controller.motor2_direction(dir_r)


        # 设置pwm
        duty_motor1 = motor_model(spd_l)
        duty_motor2 = motor_model(spd_r)
        self.sock.sendall(json.dumps({"duty_motor1":duty_motor1,"duty_motor2": duty_motor2}).encode())
        resp = self.sock.recv(256)

        print("✔ 回复:", json.loads(resp.decode()))

        # 更新里程计计算里面的轮速
        self.v_l = v_l
        self.v_r = v_r

    def cmd_to_wheel_speed(self, v):
        if abs(v) < 1e-3:
            return 0, 0.0

        direction = 1 if v > 0 else 2
        speed = abs(v) / (math.pi * WHEEL_DIAMETER)
        return direction, speed


    # =====================================================
    # odom 定时发布（新增）
    # =====================================================
    def odom_timer_cb(self):
        now = time.monotonic()
        dt = now - self.last_odom_time
        self.last_odom_time = now
        if dt <= 0.0:
            return

        with self.lock:
            v_l = self.v_l
            v_r = self.v_r

        v = (v_r + v_l) / 2.0
        w = (v_r - v_l) / WHEEL_BASE

        self.yaw += w * dt
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt

        self.publish_odom(v, w)

    def publish_odom(self, v, w):
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

    # =====================================================
    def destroy_node(self):
        self.running = False
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = CmdVelToMotor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
