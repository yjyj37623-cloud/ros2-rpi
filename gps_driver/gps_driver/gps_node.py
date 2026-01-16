#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import serial
import serial.tools.list_ports
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header

# ================= 工具函数 =================
def find_ttyUSB():
    posts = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device]
    print('当前电脑识别到的 USB 串口设备:', posts)
    return posts

KNOT_TO_MPS = 0.514444444

def NMEA_pow_n10(n): 
    return math.pow(10, -n)

def NMEA_Str2num(buf):
    buf = buf.strip()
    if not buf or buf[0] in (',', '*'):
        return 0, 0
    neg = buf.startswith('-')
    if neg: buf = buf[1:]
    if '.' in buf:
        integer, fraction = buf.split('.', 1)
    else:
        integer, fraction = buf, ''
    ilen, flen = len(integer), len(fraction)
    try:
        ires = int(integer) if integer else 0
        fres = int(fraction) if fraction else 0
    except ValueError:
        return 0, 0
    res = ires * (10 ** flen) + fres
    if neg:
        res = -res
    return res, flen

# ================= NMEA 解析 =================
class GxGGA:
    def __init__(self):
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.sat_num = 0
        self.gps_state = 0

def parse_GxGGA(buf):
    gga = GxGGA()
    if not (buf.startswith("$GPGGA") or buf.startswith("$GNGGA")):
        return None

    parts = buf.split(',')
    if len(parts) > 2 and parts[2]:
        val, dec = NMEA_Str2num(parts[2])
        tmp_double = val * NMEA_pow_n10(dec + 2)
        tmp_int = int(tmp_double)
        tmp_float = tmp_double - tmp_int
        pos_neg = -1 if parts[3] == 'S' else 1
        gga.latitude = (tmp_int + tmp_float * 1.6666666667) * pos_neg

    if len(parts) > 4 and parts[4]:
        val, dec = NMEA_Str2num(parts[4])
        tmp_double = val * NMEA_pow_n10(dec + 2)
        tmp_int = int(tmp_double)
        tmp_float = tmp_double - tmp_int
        pos_neg = -1 if parts[5] == 'W' else 1
        gga.longitude = (tmp_int + tmp_float * 1.6666666667) * pos_neg

    if len(parts) > 9 and parts[9]:
        val, dec = NMEA_Str2num(parts[9])
        gga.altitude = val * NMEA_pow_n10(dec)

    return gga

# ================= ROS2 节点 =================
class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        try:
            self.serial = serial.Serial(port, baudrate, timeout=0.5)
            self.get_logger().info(f"✅ GPS 串口打开成功: {port}")
        except Exception as e:
            self.get_logger().error(f"❌ 打开串口失败: {e}")
            raise SystemExit

        self.publisher = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.buffer = ""
        self.NMEA_HEADERS = ["$GPGGA", "$GNGGA"]
        self.timer = self.create_timer(0.2, self.timer_callback)  # 5Hz

    def timer_callback(self):
        try:
            data = self.serial.read(self.serial.in_waiting or 1).decode('ascii', errors='ignore')
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

                if header in self.NMEA_HEADERS:
                    gga = parse_GxGGA(line)
                    if gga:
                        msg = NavSatFix()
                        msg.header = Header()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = "gps_link"

                        msg.status.status = NavSatStatus.STATUS_FIX if gga.gps_state > 0 else NavSatStatus.STATUS_NO_FIX
                        msg.status.service = NavSatStatus.SERVICE_GPS

                        msg.latitude = gga.latitude
                        msg.longitude = gga.longitude
                        msg.altitude = gga.altitude
                        msg.position_covariance = [0.0]*9
                        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

                        self.publisher.publish(msg)

                        # 🔹 打印调试信息到终端
                        print(f"[DEBUG] NMEA: {line}")
                        print(f"[DEBUG] Parsed: lat={gga.latitude:.6f}, lon={gga.longitude:.6f}, alt={gga.altitude:.2f}")

        except Exception as e:
            self.get_logger().warn(f"GPS 读取异常: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GPSPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

