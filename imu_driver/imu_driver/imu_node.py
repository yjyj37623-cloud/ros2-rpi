#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import serial
import time
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler
import math
import platform
import serial.tools.list_ports

# ================= 工具函数 =================
def find_ttyUSB():
    posts = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device]
    print(f'当前电脑识别到的 USB 串口设备: {posts}')
    return posts

def checkSum(list_data, check_data):
    """校验和"""
    return sum(list_data) & 0xff == check_data

def hex_to_short(raw_data):
    """将字节数据转为 16 位整数"""
    return list(struct.unpack("hhhh", bytearray(raw_data)))

# ================= ROS2 节点 =================
class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
         
        # 参数
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 921600)
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.last_print_time = time.time()
        self.print_interval = 0.5  # 每 0.1 秒打印一次

        # 串口初始化
        try:
            self.serial = serial.Serial(port, baudrate, timeout=0.5)
            self.get_logger().info(f"✅ IMU 串口打开成功: {port}")
        except Exception as e:
            self.get_logger().error(f"❌ 串口打开失败: {e}")
            raise SystemExit

        # 发布器
        self.publisher = self.create_publisher(Imu, 'handsfree/imu', 10)

        # 数据缓存
        self.key = 0
        self.buff = {}
        self.acceleration = [0.0, 0.0, 0.0]
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.angle_degree = [0.0, 0.0, 0.0]
        self.pub_flag = [True, True, True]

        # 定时器 200Hz
        self.timer = self.create_timer(0.005, self.timer_callback)

    def handleSerialData(self, raw_byte):
        """解析串口数据并发布 Imu 消息"""
        if platform.python_version()[0] == '2':
            self.buff[self.key] = ord(raw_byte)
        else:
            self.buff[self.key] = raw_byte

        self.key += 1

        if self.buff.get(0, 0) != 0x55:
            self.key = 0
            return

        if self.key < 11:
            return

        data_buff = list(self.buff.values())
        imu_msg = Imu()
        imu_msg.header.frame_id = "base_link"

        # 加速度
        if data_buff[1] == 0x51 and self.pub_flag[0]:
            if checkSum(data_buff[0:10], data_buff[10]):
                self.acceleration = [
                    hex_to_short(data_buff[2:10])[i] / 32768.0 * 16 * 9.8 for i in range(3)
                ]
            else:
                self.get_logger().warn("0x51 校验失败")
            self.pub_flag[0] = False

        # 角速度
        elif data_buff[1] == 0x52 and self.pub_flag[1]:
            if checkSum(data_buff[0:10], data_buff[10]):
                self.angular_velocity = [
                    hex_to_short(data_buff[2:10])[i] / 32768.0 * 2000 * math.pi / 180 for i in range(3)
                ]
            else:
                self.get_logger().warn("0x52 校验失败")
            self.pub_flag[1] = False

        # 角度
        elif data_buff[1] == 0x53 and self.pub_flag[2]:
            if checkSum(data_buff[0:10], data_buff[10]):
                self.angle_degree = [
                    hex_to_short(data_buff[2:10])[i] / 32768.0 * 180 for i in range(3)
                ]
            else:
                self.get_logger().warn("0x53 校验失败")
            self.pub_flag[2] = False

        else:
            self.get_logger().warn(f"未处理数据类型 {data_buff[1]} 或数据错误")

        # 重置缓存
        self.buff = {}
        self.key = 0

        if any(self.pub_flag):
            return
        self.pub_flag = [True, True, True]

        # 填充 IMU 消息
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        angle_radian = [x * math.pi / 180 for x in self.angle_degree]
        qua = quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2])

        imu_msg.orientation.x = qua[0]
        imu_msg.orientation.y = qua[1]
        imu_msg.orientation.z = qua[2]
        imu_msg.orientation.w = qua[3]

        imu_msg.linear_acceleration.x = self.acceleration[0]
        imu_msg.linear_acceleration.y = self.acceleration[1]
        imu_msg.linear_acceleration.z = self.acceleration[2]

        imu_msg.angular_velocity.x = self.angular_velocity[0]
        imu_msg.angular_velocity.y = self.angular_velocity[1]
        imu_msg.angular_velocity.z = self.angular_velocity[2]

        self.publisher.publish(imu_msg)

        # 打印数据方便调试
        current_time = time.time()
        if current_time - self.last_print_time >= self.print_interval:
            self.get_logger().info(
                f"IMU | Acc: {self.acceleration}, Gyro: {self.angular_velocity}, Angle: {self.angle_degree}"
            )
            self.last_print_time = current_time

    def timer_callback(self):
        """定时器回调，读取串口"""
        try:
            if self.serial.in_waiting > 0:
                data = self.serial.read(self.serial.in_waiting)
                for b in data:
                    self.handleSerialData(b)
        except Exception as e:
            self.get_logger().warn(f"串口读取异常: {e}")

# ================= 主函数 =================
def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

