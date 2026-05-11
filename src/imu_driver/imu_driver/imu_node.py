#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


def checkSum(list_data, check_data):
    return (sum(list_data) & 0xff) == check_data


def hex_to_short(raw_data):
    return list(struct.unpack("hhhh", bytearray(raw_data)))


class IMUPitchNode(Node):
    def __init__(self):
        super().__init__('imu_pitch_node')

        self.declare_parameter(
            'port',
            '/dev/serial/by-id/usb-Silicon_Labs_HandsFree_IMU_USB_to_UART_Bridge_Controller_0001-if00-port0'
        )
        self.declare_parameter('baudrate', 921600)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.5)
            self.get_logger().info(f"✅ IMU 串口打开成功: {port}")
        except Exception as e:
            self.get_logger().error(f"❌ 串口打开失败: {e}")
            raise SystemExit

        # 只发布俯仰角
        self.pitch_pub = self.create_publisher(Float64, 'handsfree/pitch', 10)

        # 严格逐字节状态机
        self.buff = {}
        self.key = 0

        # 打印控制
        self.last_print_time = self.get_clock().now()
        self.print_interval = 1.0

        self.create_timer(0.002, self.read_serial)

        self.get_logger().info("🧭 IMU 俯仰角节点启动完成")

    def read_serial(self):
        if self.ser.in_waiting > 0:
            data = self.ser.read(self.ser.in_waiting)
            for b in data:
                self.handle_byte(b)

    def handle_byte(self, raw):
        self.buff[self.key] = raw
        self.key += 1

        # 帧头必须是 0x55
        if self.buff[0] != 0x55:
            self.buff = {}
            self.key = 0
            return

        # 一帧 11 字节
        if self.key < 11:
            return

        data = list(self.buff.values())

        # 只解析欧拉角帧 0x53
        if data[1] == 0x53:
            if checkSum(data[0:10], data[10]):
                angles = [
                    hex_to_short(data[2:10])[i] / 32768.0 * 180.0
                    for i in range(3)
                ]
                pitch_deg = angles[1]
                self.publish_pitch(pitch_deg)
        # 当前帧处理完，清空缓存
        self.buff = {}
        self.key = 0

    def publish_pitch(self, pitch_deg):
        msg = Float64()
        msg.data = pitch_deg
        self.pitch_pub.publish(msg)

        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_print_time).nanoseconds * 1e-9
        if time_diff >= self.print_interval:
            self.get_logger().info(f"Pitch: {pitch_deg:.2f} deg")
            self.last_print_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = IMUPitchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
