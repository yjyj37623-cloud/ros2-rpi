#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import serial
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64, Float64MultiArray


class SerialBridge(Node):
    def __init__(self):
        super().__init__('ros2_serial_bridge')

        # ========= 参数 =========
        self.declare_parameter(
            'port',
            '/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_D3091391-if00-port0'
        )
        self.declare_parameter('baud', 115200)

        # 各类数据发送频率
        self.declare_parameter('pitch_send_rate_hz', 10.0)
        self.declare_parameter('heading_send_rate_hz', 5.0)
        self.declare_parameter('gps_send_rate_hz', 5.0)
        self.declare_parameter('data_send_rate_hz', 5.0)

        # 缺失字段打印频率
        self.declare_parameter('status_log_rate_hz', 1.0)

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)

        pitch_send_rate_hz = float(self.get_parameter('pitch_send_rate_hz').value)
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

        # ========= 本机数据缓存 =========
        self.local_pitch_deg = None
        self.local_heading_rad = None
        self.local_lat = None
        self.local_lon = None
        self.local_alt = None

        # ========= 订阅本机数据 =========
        self.create_subscription(Float64, 'handsfree/pitch', self.pitch_callback, 10)
        self.create_subscription(NavSatFix, 'gps/fix', self.gps_callback, 10)
        self.create_subscription(Float64, 'gps/heading', self.heading_callback, 10)

        # ========= 发布对方数据 =========
        self.target_pitch_pub = self.create_publisher(Float64, 'target/pitch', 10)
        self.target_heading_pub = self.create_publisher(Float64, 'target/heading', 10)
        self.target_gps_pub = self.create_publisher(NavSatFix, 'target/gps', 10)
        self.target_data_pub = self.create_publisher(Float64MultiArray, 'target/data', 10)

        # ========= 串口接收缓存 =========
        self.rx_buffer = ""

        # ========= 日志节流 =========
        now = self.get_clock().now()
        self.last_pitch_send_log_time = now
        self.last_heading_send_log_time = now
        self.last_gps_send_log_time = now
        self.last_data_send_log_time = now
        self.last_recv_log_time = now
        self.log_interval = 1.0

        # ========= 定时器 =========
        self.create_timer(1.0 / pitch_send_rate_hz if pitch_send_rate_hz > 0.0 else 0.1, self.send_pitch_packet)
        self.create_timer(1.0 / heading_send_rate_hz if heading_send_rate_hz > 0.0 else 0.2, self.send_heading_packet)
        self.create_timer(1.0 / gps_send_rate_hz if gps_send_rate_hz > 0.0 else 0.2, self.send_gps_packet)
        self.create_timer(1.0 / data_send_rate_hz if data_send_rate_hz > 0.0 else 0.2, self.send_data_packet_if_ready)
        self.create_timer(0.01, self.read_serial)
        self.create_timer(1.0 / status_log_rate_hz if status_log_rate_hz > 0.0 else 1.0, self.report_missing_fields)

        self.get_logger().info("🌐 ROS2 数传节点已启动")
        self.get_logger().info("协议: PITCH / HEADING / GPS / DATA")

    # ================= 本机订阅回调 =================
    def pitch_callback(self, msg: Float64):
        self.local_pitch_deg = float(msg.data)

    def gps_callback(self, msg: NavSatFix):
        self.local_lat = float(msg.latitude)
        self.local_lon = float(msg.longitude)
        self.local_alt = float(msg.altitude)

    def heading_callback(self, msg: Float64):
        self.local_heading_rad = float(msg.data)

    # ================= 单项发送 =================
    def send_pitch_packet(self):
        if self.local_pitch_deg is None:
            return

        text = f"PITCH,{self.local_pitch_deg:.2f}\n"
        try:
            self.ser.write(text.encode('utf-8'))
            self.maybe_log_pitch_send(text.strip())
        except Exception as e:
            self.get_logger().warn(f"串口发送 PITCH 失败: {e}")

    def send_heading_packet(self):
        if self.local_heading_rad is None:
            return

        heading_deg = math.degrees(self.local_heading_rad)
        text = f"HEADING,{heading_deg:.2f}\n"
        try:
            self.ser.write(text.encode('utf-8'))
            self.maybe_log_heading_send(text.strip())
        except Exception as e:
            self.get_logger().warn(f"串口发送 HEADING 失败: {e}")

    def send_gps_packet(self):
        if self.local_lat is None or self.local_lon is None or self.local_alt is None:
            return

        text = f"GPS,{self.local_lat:.6f},{self.local_lon:.6f},{self.local_alt:.2f}\n"
        try:
            self.ser.write(text.encode('utf-8'))
            self.maybe_log_gps_send(text.strip())
        except Exception as e:
            self.get_logger().warn(f"串口发送 GPS 失败: {e}")

    # ================= 完整 DATA 发送 =================
    def send_data_packet_if_ready(self):
        missing = self.get_missing_fields_for_data()
        if missing:
            return

        heading_deg = math.degrees(self.local_heading_rad)
        text = (
            f"DATA,"
            f"{self.local_pitch_deg:.2f},"
            f"{heading_deg:.2f},"
            f"{self.local_lat:.6f},"
            f"{self.local_lon:.6f},"
            f"{self.local_alt:.2f}\n"
        )

        try:
            self.ser.write(text.encode('utf-8'))
            self.maybe_log_data_send(text.strip())
        except Exception as e:
            self.get_logger().warn(f"串口发送 DATA 失败: {e}")

    def get_missing_fields_for_data(self):
        missing = []
        if self.local_pitch_deg is None:
            missing.append('pitch')
        if self.local_heading_rad is None:
            missing.append('heading')
        if self.local_lat is None:
            missing.append('latitude')
        if self.local_lon is None:
            missing.append('longitude')
        if self.local_alt is None:
            missing.append('altitude')
        return missing

    def report_missing_fields(self):
        missing = self.get_missing_fields_for_data()
        if missing:
            self.get_logger().info(f"等待完整 DATA 数据源: {', '.join(missing)} missing")

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

                if line.startswith("PITCH,"):
                    self.parse_target_pitch(line)
                elif line.startswith("HEADING,"):
                    self.parse_target_heading(line)
                elif line.startswith("GPS,"):
                    self.parse_target_gps(line)
                elif line.startswith("DATA,"):
                    self.parse_target_data(line)

        except Exception as e:
            self.get_logger().warn(f"串口接收异常: {e}")

    # ================= 解析：对方单项数据 =================
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

            self.maybe_log_recv(f"PITCH recv: {pitch_deg:.2f} deg")

        except Exception as e:
            self.get_logger().warn(f"解析目标 PITCH 失败: {e} | 原始数据: {line}")

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

            self.maybe_log_recv(f"HEADING recv: {heading_deg:.2f} deg")

        except Exception as e:
            self.get_logger().warn(f"解析目标 HEADING 失败: {e} | 原始数据: {line}")

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
            self.get_logger().warn(f"解析目标 GPS 失败: {e} | 原始数据: {line}")

    # ================= 解析：对方完整 DATA =================
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

            # 2) target/heading
            heading_msg = Float64()
            heading_msg.data = math.radians(heading_deg)
            self.target_heading_pub.publish(heading_msg)

            # 3) target/gps
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
                f"DATA recv: pitch={pitch_deg:.2f} deg, "
                f"heading={heading_deg:.2f} deg, "
                f"lat={lat:.6f}, lon={lon:.6f}, alt={alt:.2f}"
            )

        except Exception as e:
            self.get_logger().warn(f"解析目标 DATA 失败: {e} | 原始数据: {line}")

    # ================= 日志节流 =================
    def maybe_log_pitch_send(self, text: str):
        now = self.get_clock().now()
        if (now - self.last_pitch_send_log_time).nanoseconds * 1e-9 >= self.log_interval:
            self.get_logger().info(f"➡️ 发送: {text}")
            self.last_pitch_send_log_time = now

    def maybe_log_heading_send(self, text: str):
        now = self.get_clock().now()
        if (now - self.last_heading_send_log_time).nanoseconds * 1e-9 >= self.log_interval:
            self.get_logger().info(f"➡️ 发送: {text}")
            self.last_heading_send_log_time = now

    def maybe_log_gps_send(self, text: str):
        now = self.get_clock().now()
        if (now - self.last_gps_send_log_time).nanoseconds * 1e-9 >= self.log_interval:
            self.get_logger().info(f"➡️ 发送: {text}")
            self.last_gps_send_log_time = now

    def maybe_log_data_send(self, text: str):
        now = self.get_clock().now()
        if (now - self.last_data_send_log_time).nanoseconds * 1e-9 >= self.log_interval:
            self.get_logger().info(f"➡️ 发送: {text}")
            self.last_data_send_log_time = now

    def maybe_log_recv(self, text: str):
        now = self.get_clock().now()
        if (now - self.last_recv_log_time).nanoseconds * 1e-9 >= self.log_interval:
            self.get_logger().info(f"⬅️ 接收: {text}")
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
