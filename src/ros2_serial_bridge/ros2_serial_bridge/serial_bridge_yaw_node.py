#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import serial
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64, Float64MultiArray


class SerialBridgeYaw(Node):
    def __init__(self):
        super().__init__('serial_bridge_yaw_node')

        # ========= 参数 =========
        self.declare_parameter(
            'port',
            '/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_D3091391-if00-port0'
        )
        self.declare_parameter('baud', 115200)

        self.declare_parameter('heading_send_rate_hz', 5.0)
        self.declare_parameter('gps_send_rate_hz', 5.0)
        self.declare_parameter('data_send_rate_hz', 5.0)
        self.declare_parameter('status_log_rate_hz', 1.0)

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)

        heading_send_rate_hz = float(self.get_parameter('heading_send_rate_hz').value)
        gps_send_rate_hz = float(self.get_parameter('gps_send_rate_hz').value)
        data_send_rate_hz = float(self.get_parameter('data_send_rate_hz').value)
        status_log_rate_hz = float(self.get_parameter('status_log_rate_hz').value)

        # ========= 串口 =========
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"✅ 串口打开成功: {port}")
        except Exception as e:
            self.get_logger().error(f"❌ 串口打开失败: {e}")
            raise SystemExit

        # ========= 本机数据 =========
        self.local_heading_rad = None
        self.local_lat = None
        self.local_lon = None
        self.local_alt = None

        # ========= 订阅 =========
        self.create_subscription(Float64, 'gps/heading', self.heading_callback, 10)
        self.create_subscription(NavSatFix, 'gps/fix', self.gps_callback, 10)

        # ========= 发布 =========
        self.target_heading_pub = self.create_publisher(Float64, 'target/heading', 10)
        self.target_gps_pub = self.create_publisher(NavSatFix, 'target/gps', 10)
        self.target_data_pub = self.create_publisher(Float64MultiArray, 'target/data', 10)

        # ========= 缓存 =========
        self.rx_buffer = ""

        # ========= 定时器 =========
        self.create_timer(1.0 / heading_send_rate_hz, self.send_heading_packet)
        self.create_timer(1.0 / gps_send_rate_hz, self.send_gps_packet)
        self.create_timer(1.0 / data_send_rate_hz, self.send_data_packet_if_ready)
        self.create_timer(0.01, self.read_serial)
        self.create_timer(1.0 / status_log_rate_hz, self.report_missing_fields)

        self.get_logger().info("🌐 YAW-only 数传节点已启动")
        self.get_logger().info("协议: HEADING / GPS / DATA(yaw-only)")

    # ================= 回调 =================
    def heading_callback(self, msg: Float64):
        self.local_heading_rad = float(msg.data)

    def gps_callback(self, msg: NavSatFix):
        self.local_lat = float(msg.latitude)
        self.local_lon = float(msg.longitude)
        self.local_alt = float(msg.altitude)

    # ================= 发送 =================
    def send_heading_packet(self):
        if self.local_heading_rad is None:
            return

        heading_deg = math.degrees(self.local_heading_rad)
        text = f"HEADING,{heading_deg:.2f}\n"

        self.ser.write(text.encode())

    def send_gps_packet(self):
        if None in (self.local_lat, self.local_lon, self.local_alt):
            return

        text = f"GPS,{self.local_lat:.6f},{self.local_lon:.6f},{self.local_alt:.2f}\n"
        self.ser.write(text.encode())

    def send_data_packet_if_ready(self):
        if self.local_heading_rad is None:
            return
        if None in (self.local_lat, self.local_lon, self.local_alt):
            return

        heading_deg = math.degrees(self.local_heading_rad)

        # ✅ only_yaw 协议
        text = (
            f"DATA,"
            f"{heading_deg:.2f},"
            f"{self.local_lat:.6f},"
            f"{self.local_lon:.6f},"
            f"{self.local_alt:.2f}\n"
        )

        self.ser.write(text.encode())

    def report_missing_fields(self):
        missing = []
        if self.local_heading_rad is None:
            missing.append("heading")
        if self.local_lat is None:
            missing.append("lat")
        if self.local_lon is None:
            missing.append("lon")
        if self.local_alt is None:
            missing.append("alt")

        if missing:
            self.get_logger().info(f"等待数据: {missing}")

    # ================= 接收 =================
    def read_serial(self):
        data = self.ser.read(self.ser.in_waiting or 1).decode(errors='ignore')
        if not data:
            return

        self.rx_buffer += data

        while '\n' in self.rx_buffer:
            line, self.rx_buffer = self.rx_buffer.split('\n', 1)
            line = line.strip()

            if line.startswith("HEADING,"):
                self.parse_target_heading(line)
            elif line.startswith("GPS,"):
                self.parse_target_gps(line)
            elif line.startswith("DATA,"):
                self.parse_target_data(line)

    def parse_target_heading(self, line):
        parts = line.split(',')
        heading_deg = float(parts[1])

        msg = Float64()
        msg.data = math.radians(heading_deg)
        self.target_heading_pub.publish(msg)

    def parse_target_gps(self, line):
        parts = line.split(',')
        lat, lon, alt = float(parts[1]), float(parts[2]), float(parts[3])

        msg = NavSatFix()
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt
        self.target_gps_pub.publish(msg)

    def parse_target_data(self, line):
        parts = line.split(',')

        # ✅ only_yaw：5个字段
        if len(parts) != 5:
            self.get_logger().warn(f"DATA格式错误: {line}")
            return

        heading_deg = float(parts[1])
        lat = float(parts[2])
        lon = float(parts[3])
        alt = float(parts[4])

        # heading
        h_msg = Float64()
        h_msg.data = math.radians(heading_deg)
        self.target_heading_pub.publish(h_msg)

        # gps
        gps_msg = NavSatFix()
        gps_msg.latitude = lat
        gps_msg.longitude = lon
        gps_msg.altitude = alt
        self.target_gps_pub.publish(gps_msg)

        # data
        data_msg = Float64MultiArray()
        data_msg.data = [heading_deg, lat, lon, alt]
        self.target_data_pub.publish(data_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeYaw()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()