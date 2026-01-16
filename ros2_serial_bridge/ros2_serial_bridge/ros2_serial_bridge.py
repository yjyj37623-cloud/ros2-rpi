#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import serial
import math

from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Quaternion
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class SerialBridge(Node):
    def __init__(self):
        super().__init__('ros2_serial_bridge')

        # ========= 参数 =========
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        # ========= 串口 =========
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"✅ 串口打开成功: {port}")
        except Exception as e:
            self.get_logger().error(f"❌ 串口打开失败: {e}")
            raise SystemExit

        # ========= 订阅（发送给对方） =========
        self.create_subscription(Imu, 'handsfree/imu', self.imu_callback, 10)
        self.create_subscription(NavSatFix, 'gps/fix', self.gps_callback, 10)

        # ========= 发布（接收自对方） =========
        self.target_imu_pub = self.create_publisher(Imu, 'target/imu', 10)
        self.target_gps_pub = self.create_publisher(NavSatFix, 'target/gps', 10)

        # ========= 串口接收 =========
        self.buffer = ""
        self.create_timer(0.01, self.read_serial)  # 100 Hz 轮询

        self.get_logger().info("🌐 ROS2 数传节点已启动")

    # ================= 发送部分 =================
    def imu_callback(self, msg: Imu):
        q = msg.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        imu_str = (
            f"IMU,"
            f"{math.degrees(roll):.2f},"
            f"{math.degrees(pitch):.2f},"
            f"{math.degrees(yaw):.2f},"
            f"{msg.angular_velocity.x:.3f},"
            f"{msg.angular_velocity.y:.3f},"
            f"{msg.angular_velocity.z:.3f}\n"
        )

        self.send_serial(imu_str)

    def gps_callback(self, msg: NavSatFix):
        gps_str = (
            f"GPS,"
            f"{msg.latitude:.6f},"
            f"{msg.longitude:.6f},"
            f"{msg.altitude:.2f}\n"
        )
        self.send_serial(gps_str)

    def send_serial(self, text: str):
        try:
            self.ser.write(text.encode('utf-8'))
            self.get_logger().debug(f"➡️ 串口发送: {text.strip()}")
        except Exception as e:
            self.get_logger().warn(f"串口发送失败: {e}")

    # ================= 接收部分 =================
    def read_serial(self):
        try:
            data = self.ser.read(self.ser.in_waiting or 1).decode('utf-8', errors='ignore')
            if not data:
                return

            self.buffer += data

            while '\n' in self.buffer:
                line, self.buffer = self.buffer.split('\n', 1)
                line = line.strip()

                if line.startswith("IMU"):
                    self.parse_target_imu(line)
                elif line.startswith("GPS"):
                    self.parse_target_gps(line)

        except Exception as e:
            self.get_logger().warn(f"串口接收异常: {e}")

    # ================= 解析部分 =================
    def parse_target_imu(self, line: str):
        try:
            parts = line.split(',')
            if len(parts) != 7:
                return

            roll = math.radians(float(parts[1]))
            pitch = math.radians(float(parts[2]))
            yaw = math.radians(float(parts[3]))
            wx, wy, wz = map(float, parts[4:7])

            qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "target_base_link"
            msg.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
            msg.angular_velocity.x = wx
            msg.angular_velocity.y = wy
            msg.angular_velocity.z = wz

            self.target_imu_pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"解析目标 IMU 失败: {e}")

    def parse_target_gps(self, line: str):
        try:
            parts = line.split(',')
            if len(parts) != 4:
                return

            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "target_gps"
            msg.latitude = float(parts[1])
            msg.longitude = float(parts[2])
            msg.altitude = float(parts[3])

            self.target_gps_pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"解析目标 GPS 失败: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

