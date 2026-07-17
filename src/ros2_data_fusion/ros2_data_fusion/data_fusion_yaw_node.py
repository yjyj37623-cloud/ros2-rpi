#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64, Float64MultiArray

EARTH_RADIUS = 6371000.0  # m


# ================= 工具函数 =================
def get_bearing_point_2_point_NED(current, target):
    lat1, lon1 = math.radians(current['lat']), math.radians(current['lon'])
    lat2, lon2 = math.radians(target['lat']), math.radians(target['lon'])
    dlon = lon2 - lon1

    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)

    brng = math.atan2(y, x)
    return (math.degrees(brng) + 360.0) % 360.0


def wrap_angle(angle):
    angle = angle % 360.0
    if angle > 180.0:
        angle -= 360.0
    return angle


# ================= 主节点 =================
class DataFusionYawNode(Node):
    def __init__(self):
        super().__init__('data_fusion_yaw_node')

        # ========= 本机订阅 =========
        self.create_subscription(NavSatFix, 'gps/fix', self.local_gps_callback, 10)
        self.create_subscription(Float64, 'gps/heading', self.local_heading_callback, 10)

        # ========= 对方订阅 =========
        # only_yaw 协议：target/data = [heading_deg, lat, lon, alt]
        self.create_subscription(Float64MultiArray, 'target/data', self.target_data_callback, 10)

        # ========= 发布 =========
        self.pub_gimbal_cmd = self.create_publisher(Vector3, '/track/gimbal_cmd', 10)
        self.pub_target_angles = self.create_publisher(Vector3, 'target/angles', 10)

        # ========= 状态量 =========
        self.current_gps = None
        self.current_heading_deg = None

        self.target_heading_deg = None
        self.target_gps = None

        # ========= 控制参数 =========
        self.declare_parameter('yaw_kp', 0.2)
        self.declare_parameter('yaw_vel_limit', 15.0)
        self.declare_parameter('yaw_deadband_deg', 0.1)
        self.declare_parameter('yaw_min_vel', 0.20)

        # 如果 pan 方向反了，只改这个参数
        self.declare_parameter('yaw_dir', -1.0)

        # heading 偏置补偿
        self.declare_parameter('heading_offset_deg', 0.0)

        self.yaw_kp = float(self.get_parameter('yaw_kp').value)
        self.yaw_vel_limit = float(self.get_parameter('yaw_vel_limit').value)
        self.yaw_deadband_deg = float(self.get_parameter('yaw_deadband_deg').value)
        self.yaw_min_vel = float(self.get_parameter('yaw_min_vel').value)
        self.yaw_dir = float(self.get_parameter('yaw_dir').value)
        self.heading_offset_deg = float(self.get_parameter('heading_offset_deg').value)

        # ========= 日志计时 =========
        self.last_print_time = self.get_clock().now()

        # ========= 控制定时器 =========
        self.create_timer(0.02, self.run_control)

        self.get_logger().info("🧠 data_fusion_yaw_node 已启动（only_yaw 模式）")

    # ========= 回调 =========
    def local_gps_callback(self, msg: NavSatFix):
        self.current_gps = {
            'lat': float(msg.latitude),
            'lon': float(msg.longitude),
            'alt': float(msg.altitude)
        }

    def local_heading_callback(self, msg: Float64):
        # 保持和你原工程一致：上游 heading 默认是弧度
        self.current_heading_deg = (math.degrees(float(msg.data)) + 360.0) % 360.0

    def target_data_callback(self, msg: Float64MultiArray):
        # only_yaw: [heading_deg, lat, lon, alt]
        if len(msg.data) != 4:
            self.get_logger().warn(f"target/data 长度错误(only_yaw 应为4): {len(msg.data)}")
            return

        self.target_heading_deg = float(msg.data[0])
        self.target_gps = {
            'lat': float(msg.data[1]),
            'lon': float(msg.data[2]),
            'alt': float(msg.data[3]),
        }

    # ========= 控制工具 =========
    def clamp(self, value, limit_abs):
        return max(min(value, limit_abs), -limit_abs)

    def calc_p_output(self, error, kp, deadband, min_vel, vel_limit):
        if abs(error) < deadband:
            return 0.0

        vel = kp * error
        vel = self.clamp(vel, vel_limit)

        if 0.0 < abs(vel) < min_vel:
            vel = math.copysign(min_vel, vel)

        return vel

    # ========= 控制逻辑 =========
    def run_control(self):
        if self.current_gps is None or self.current_heading_deg is None or self.target_gps is None:
            return

        # 目标方位角
        bearing = get_bearing_point_2_point_NED(self.current_gps, self.target_gps)

        # 当前朝向
        antenna_heading = (self.current_heading_deg + self.heading_offset_deg) % 360.0

        # ========================
        # ✅ 修复：正确最短路径误差
        # ========================
        yaw_error = wrap_angle(bearing - antenna_heading)

        # 仅控制 yaw
        yaw_vel = self.calc_p_output(
            error=yaw_error,
            kp=self.yaw_kp,
            deadband=self.yaw_deadband_deg,
            min_vel=self.yaw_min_vel,
            vel_limit=self.yaw_vel_limit
        )

        yaw_vel *= self.yaw_dir

        # 发布控制命令
        cmd_msg = Vector3()
        cmd_msg.x = yaw_vel
        cmd_msg.y = 0.0
        cmd_msg.z = 0.0
        self.pub_gimbal_cmd.publish(cmd_msg)

        # 调试
        dbg_msg = Vector3()
        dbg_msg.x = bearing
        dbg_msg.y = antenna_heading
        dbg_msg.z = yaw_error
        self.pub_target_angles.publish(dbg_msg)

        # 日志
        now = self.get_clock().now()
        if (now - self.last_print_time).nanoseconds * 1e-9 >= 1.0:
            self.get_logger().info(
                f"[控制] target_yaw={bearing:.2f} | "
                f"heading={self.current_heading_deg:.2f}, antenna_heading={antenna_heading:.2f} | "
                f"yaw_error={yaw_error:.2f} | "
                f"yaw_vel={yaw_vel:.3f}"
            )
            self.last_print_time = now


def main(args=None):
    rclpy.init(args=args)
    node = DataFusionYawNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

