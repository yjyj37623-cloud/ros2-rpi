#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import serial
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64, Float64MultiArray, Header
from geometry_msgs.msg import Vector3Stamped


class SerialBridge(Node):
    def __init__(self):
        super().__init__('ros2_serial_bridge')

        # ========= 参数 =========
        self.declare_parameter(
            'port',
            '/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_D3091391-if00-port0'
        )
        self.declare_parameter('baud', 115200)
        self.declare_parameter('status_log_rate_hz', 1.0)

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        status_log_rate_hz = float(self.get_parameter('status_log_rate_hz').value)

        # ========= 串口 =========
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"串口打开成功: {port}")
        except Exception as e:
            self.get_logger().error(f"串口打开失败: {e}")
            raise SystemExit

        # ========= 发布对方数据 (原有话题，不变) =========
        self.target_pitch_pub = self.create_publisher(Float64, 'target/pitch', 10)
        self.target_heading_pub = self.create_publisher(Float64, 'target/heading', 10)
        self.target_gps_pub = self.create_publisher(NavSatFix, 'target/gps', 10)
        self.target_data_pub = self.create_publisher(Float64MultiArray, 'target/data', 10)
        self.target_yaw_error_rx_pub = self.create_publisher(Float64, 'target/yaw_error_rx', 10)

        # ========= 发布对方数据 (新话题，带 Header.stamp) =========
        self.target_pitch_stamped_pub = self.create_publisher(Vector3Stamped, 'target/pitch_stamped', 10)
        self.target_heading_stamped_pub = self.create_publisher(Vector3Stamped, 'target/heading_deg_stamped', 10)
        self.target_yaw_error_stamped_pub = self.create_publisher(Vector3Stamped, 'target/yaw_error_rx_stamped', 10)

        # ========= 串口接收缓存 =========
        self.rx_buffer = ""

        # ========= 日志节流 =========
        self.last_recv_log_time = self.get_clock().now()
        self.log_interval = 1.0

        # ========= 定时器 =========
        self.create_timer(0.01, self.read_serial)
        self.create_timer(1.0 / status_log_rate_hz if status_log_rate_hz > 0.0 else 1.0, self.status_log)

        self.get_logger().info("ROS2 数传接收节点已启动 (仅接收)")
        self.get_logger().info("协议: PITCH / HEADING / GPS / DATA / YERR")

    # ================= 串口接收 =================
    def read_serial(self):
        try:
            data = self.ser.read(self.ser.in_waiting or 1).decode('utf-8', errors='ignore')
            if not data:
                return

            self.rx_buffer += data

            while '\n' in self.rx_buffer:
                line, self.rx_buffer = self.rx_buffer.split('\n', 1)
                line = line.strip()

                if not line:
                    continue

                if line.startswith("YERR,"):
                    self.parse_target_yaw_error(line)
                elif line.startswith("PITCH,"):
                    self.parse_target_pitch(line)
                elif line.startswith("HEADING,"):
                    self.parse_target_heading(line)
                elif line.startswith("GPS,"):
                    self.parse_target_gps(line)
                elif line.startswith("DATA,"):
                    self.parse_target_data(line)

        except Exception as e:
            self.get_logger().warn(f"串口接收异常: {e}")

    # ================= 解析：PITCH =================
    def parse_target_pitch(self, line: str):
        try:
            parts = line.split(',')
            if len(parts) != 2:
                self.get_logger().warn(f"PITCH 协议字段数错误: {line}")
                return

            pitch_deg = float(parts[1])

            msg = Float64()
            msg.data = pitch_deg
            self.target_pitch_pub.publish(msg)

            smsg = self._make_stamped(x=pitch_deg)
            self.target_pitch_stamped_pub.publish(smsg)

            self.maybe_log_recv(f"PITCH recv: {pitch_deg:.2f} deg")

        except Exception as e:
            self.get_logger().warn(f"解析 PITCH 失败: {e} | 原始数据: {line}")

    # ================= 解析：HEADING =================
    def parse_target_heading(self, line: str):
        try:
            parts = line.split(',')
            if len(parts) != 2:
                self.get_logger().warn(f"HEADING 协议字段数错误: {line}")
                return

            heading_deg = float(parts[1])

            msg = Float64()
            msg.data = math.radians(heading_deg)
            self.target_heading_pub.publish(msg)

            smsg = self._make_stamped(x=heading_deg)
            self.target_heading_stamped_pub.publish(smsg)

            self.maybe_log_recv(f"HEADING recv: {heading_deg:.2f} deg")

        except Exception as e:
            self.get_logger().warn(f"解析 HEADING 失败: {e} | 原始数据: {line}")

    # ================= 解析：GPS =================
    def parse_target_gps(self, line: str):
        try:
            parts = line.split(',')
            if len(parts) != 4:
                self.get_logger().warn(f"GPS 协议字段数错误: {line}")
                return

            lat = float(parts[1])
            lon = float(parts[2])
            alt = float(parts[3])

            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "target_gps"
            msg.status.status = NavSatStatus.STATUS_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS
            msg.latitude = lat
            msg.longitude = lon
            msg.altitude = alt
            msg.position_covariance = [0.0] * 9
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            self.target_gps_pub.publish(msg)

            self.maybe_log_recv(f"GPS recv: lat={lat:.6f}, lon={lon:.6f}, alt={alt:.2f}")

        except Exception as e:
            self.get_logger().warn(f"解析 GPS 失败: {e} | 原始数据: {line}")

    # ================= 解析：DATA =================
    def parse_target_data(self, line: str):
        try:
            parts = line.split(',')
            if len(parts) != 6:
                self.get_logger().warn(f"DATA 协议字段数错误: {line}")
                return

            pitch_deg = float(parts[1])
            heading_deg = float(parts[2])
            lat = float(parts[3])
            lon = float(parts[4])
            alt = float(parts[5])

            # 1) target/pitch
            pitch_msg = Float64()
            pitch_msg.data = pitch_deg
            self.target_pitch_pub.publish(pitch_msg)
            self.target_pitch_stamped_pub.publish(self._make_stamped(x=pitch_deg))

            # 2) target/heading
            heading_msg = Float64()
            heading_msg.data = math.radians(heading_deg)
            self.target_heading_pub.publish(heading_msg)
            self.target_heading_stamped_pub.publish(self._make_stamped(x=heading_deg))

            # 3) target/gps (NavSatFix 自带 header.stamp)
            gps_msg = NavSatFix()
            gps_msg.header.stamp = self.get_clock().now().to_msg()
            gps_msg.header.frame_id = "target_gps"
            gps_msg.status.status = NavSatStatus.STATUS_FIX
            gps_msg.status.service = NavSatStatus.SERVICE_GPS
            gps_msg.latitude = lat
            gps_msg.longitude = lon
            gps_msg.altitude = alt
            gps_msg.position_covariance = [0.0] * 9
            gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            self.target_gps_pub.publish(gps_msg)

            # 4) target/data
            data_msg = Float64MultiArray()
            data_msg.data = [pitch_deg, heading_deg, lat, lon, alt]
            self.target_data_pub.publish(data_msg)

            self.maybe_log_recv(
                f"DATA recv: pitch={pitch_deg:.2f}, heading={heading_deg:.2f}, "
                f"lat={lat:.6f}, lon={lon:.6f}, alt={alt:.2f}"
            )

        except Exception as e:
            self.get_logger().warn(f"解析 DATA 失败: {e} | 原始数据: {line}")

    # ================= 解析：YERR =================
    def parse_target_yaw_error(self, line: str):
        try:
            parts = line.split(',')
            if len(parts) != 2:
                self.get_logger().warn(f"YERR 协议字段数错误: {line}")
                return

            err_val = float(parts[1])

            msg = Float64()
            msg.data = err_val
            self.target_yaw_error_rx_pub.publish(msg)

            self.target_yaw_error_stamped_pub.publish(self._make_stamped(x=err_val))

            self.maybe_log_recv(f"YERR recv: {err_val:.3f} deg")

        except Exception as e:
            self.get_logger().warn(f"解析 YERR 失败: {e} | 原始数据: {line}")

    # ================= 辅助：生成带时间戳的消息 =================
    def _make_stamped(self, x=0.0, y=0.0, z=0.0):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vector.x = x
        msg.vector.y = y
        msg.vector.z = z
        return msg

    # ================= 状态日志 =================
    def status_log(self):
        self.get_logger().info("数传接收节点运行中...")

    # ================= 日志节流 =================
    def maybe_log_recv(self, text: str):
        now = self.get_clock().now()
        if (now - self.last_recv_log_time).nanoseconds * 1e-9 >= self.log_interval:
            self.get_logger().info(f"接收: {text}")
            self.last_recv_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
