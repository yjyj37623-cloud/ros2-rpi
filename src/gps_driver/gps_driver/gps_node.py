#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import serial
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header, Float64, Bool
# --- 新增引用 ---
from geometry_msgs.msg import Vector3Stamped

# ================= 工具函数 =================
def NMEA_pow_n10(n):
    return math.pow(10, -n)

def NMEA_Str2num(buf):
    buf = buf.strip()
    if not buf or buf[0] in (',', '*'):
        return 0, 0

    neg = buf.startswith('-')
    if neg:
        buf = buf[1:]

    if '.' in buf:
        integer, fraction = buf.split('.', 1)
    else:
        integer, fraction = buf, ''

    try:
        ires = int(integer) if integer else 0
        fres = int(fraction) if fraction else 0
    except ValueError:
        return 0, 0

    flen = len(fraction)
    res = ires * (10 ** flen) + fres
    if neg:
        res = -res
    return res, flen

# ================= GGA 解析 =================
def parse_GxGGA(buf):
    if not (buf.startswith("$GPGGA") or buf.startswith("$GNGGA")):
        return None

    parts = buf.split(',')
    if len(parts) < 10:
        return None

    latitude = 0.0
    longitude = 0.0
    altitude = 0.0
    gps_state = 0

    # latitude
    if parts[2]:
        val, dec = NMEA_Str2num(parts[2])
        tmp = val * NMEA_pow_n10(dec + 2)
        ideg = int(tmp)
        frac = tmp - ideg
        pos_neg = -1 if parts[3] == 'S' else 1
        latitude = (ideg + frac * 1.6666666667) * pos_neg

    # longitude
    if parts[4]:
        val, dec = NMEA_Str2num(parts[4])
        tmp = val * NMEA_pow_n10(dec + 2)
        ideg = int(tmp)
        frac = tmp - ideg
        pos_neg = -1 if parts[5] == 'W' else 1
        longitude = (ideg + frac * 1.6666666667) * pos_neg

    # fix state
    if parts[6]:
        try:
            gps_state = int(parts[6])
        except ValueError:
            gps_state = 0

    # altitude
    if parts[9]:
        val, dec = NMEA_Str2num(parts[9])
        altitude = val * NMEA_pow_n10(dec)

    return {
        'latitude': latitude,
        'longitude': longitude,
        'altitude': altitude,
        'gps_state': gps_state,
    }

# ================= HEADINGA 解析 =================
def parse_HEADINGA(buf):
    if not buf.startswith("#HEADINGA"):
        return None

    parts = buf.split(',')
    if len(parts) < 13:
        return None

    try:
        heading_deg = float(parts[12].split('*')[0])
    except ValueError:
        return None

    return heading_deg

# ================= ROS2 节点 =================
class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')

        self.declare_parameter(
            'port',
            '/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_CAABb13AL20-if00-port0'
        )
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        try:
            self.serial = serial.Serial(port, baudrate, timeout=0.5)
            self.get_logger().info(f"✅ GPS 串口打开成功: {port} {baudrate}")
        except Exception as e:
            self.get_logger().error(f"❌ 打开串口失败: {e}")
            raise SystemExit

        # 1. 原有发布者：经纬度
        self.fix_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)
        
        # 2. 原有发布者：弧度航向角 (保持不变，供旧代码使用)
        self.heading_pub = self.create_publisher(Float64, 'gps/heading', 10)
        
        # 3. 原有发布者：定位状态
        self.fix_status_pub = self.create_publisher(Bool, 'gps/fix_status', 10)

        # --- 新增发布者：度单位航向角 (带时间戳) ---
        # 使用 Vector3 消息类型，x 存角度，Header 存时间戳
        self.heading_deg_pub = self.create_publisher(Vector3Stamped, 'gps/heading_deg', 10)

        self.buffer = ""
        self.NMEA_HEADERS = ["$GPGGA", "$GNGGA", "#HEADINGA"]

        self.last_print_time = self.get_clock().now()
        self.print_interval = 1.0

        self.latest_heading_deg = None
        self.latest_gps = None

        self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("📡 GPS 节点已启动")

    def timer_callback(self):
        try:
            data = self.serial.read(self.serial.in_waiting or 1).decode('ascii', errors='ignore')
            if not data:
                return

            self.buffer += data

            while True:
                start_idx = -1
                header = None

                for h in self.NMEA_HEADERS:
                    idx = self.buffer.find(h)
                    if idx != -1 and (start_idx == -1 or idx < start_idx):
                        start_idx = idx
                        header = h

                if start_idx == -1:
                    break

                end_idx = self.buffer.find('\n', start_idx)
                if end_idx == -1:
                    break

                line = self.buffer[start_idx:end_idx].strip()
                self.buffer = self.buffer[end_idx + 1:]

                # ===== GGA：经纬度高度 =====
                if header in ("$GPGGA", "$GNGGA"):
                    gga = parse_GxGGA(line)
                    if gga is not None:
                        fix_ok = gga['gps_state'] > 0
                        self.fix_status_pub.publish(Bool(data=fix_ok))

                        msg = NavSatFix()
                        msg.header = Header()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = "gps_link"
                        msg.status.status = (
                            NavSatStatus.STATUS_FIX if fix_ok else NavSatStatus.STATUS_NO_FIX
                        )
                        msg.status.service = NavSatStatus.SERVICE_GPS
                        msg.latitude = gga['latitude']
                        msg.longitude = gga['longitude']
                        msg.altitude = gga['altitude']
                        msg.position_covariance = [0.0] * 9
                        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

                        self.fix_pub.publish(msg)
                        self.latest_gps = gga

                        self.maybe_print()

                # ===== HEADINGA：航向角 =====
                elif header == "#HEADINGA":
                    heading_deg = parse_HEADINGA(line)
                    if heading_deg is not None:
                        # --- 1. 发布原有的弧度消息 (保持不变) ---
                        self.heading_pub.publish(Float64(data=math.radians(heading_deg)))
                        
                        # --- 2. 发布新的度消息 (带时间戳) ---
                        msg_deg = Vector3Stamped()
                        msg_deg.header.stamp = self.get_clock().now().to_msg()
                        msg_deg.header.frame_id = "gps_link"
                        msg_deg.vector.x = heading_deg
                        self.heading_deg_pub.publish(msg_deg)

                        self.latest_heading_deg = heading_deg

                        self.maybe_print()

        except Exception as e:
            self.get_logger().warn(f"GPS 读取异常: {e}")

    def maybe_print(self):
        now = self.get_clock().now()
        dt = (now - self.last_print_time).nanoseconds * 1e-9
        if dt < self.print_interval:
            return

        gps_str = "lat=--, lon=--, alt=--"
        if self.latest_gps is not None:
            gps_str = (
                f"lat={self.latest_gps['latitude']:.6f}, "
                f"lon={self.latest_gps['longitude']:.6f}, "
                f"alt={self.latest_gps['altitude']:.2f}m"
            )

        heading_str = "--"
        if self.latest_heading_deg is not None:
            heading_str = f"{self.latest_heading_deg:.4f}°"

        self.get_logger().info(f"📍 {gps_str} | 🧭 heading={heading_str}")
        self.last_print_time = now

def main(args=None):
    rclpy.init(args=args)
    node = GPSPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
