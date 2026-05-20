#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3, Vector3Stamped
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


def get_pitch_point_2_point_NED(current, target):
    dlat = math.radians(target['lat'] - current['lat'])
    dlon = math.radians(target['lon'] - current['lon'])

    d_north = dlat * EARTH_RADIUS
    d_east = dlon * EARTH_RADIUS * math.cos(math.radians(current['lat']))
    d_alt = target['alt'] - current['alt']

    horizontal = math.sqrt(d_north ** 2 + d_east ** 2)
    return math.degrees(math.atan2(d_alt, horizontal))


def wrap_angle(angle):
    while angle >= 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


# ================= 主节点 =================
class DataFusionNode(Node):
    def __init__(self):
        super().__init__('data_fusion_node')

        # ========= 本机订阅 =========
        self.create_subscription(Float64, 'handsfree/pitch', self.local_pitch_callback, 10)
        self.create_subscription(NavSatFix, 'gps/fix', self.local_gps_callback, 10)
        self.create_subscription(Float64, 'gps/heading', self.local_heading_callback, 10)

        # ========= 对方订阅 =========
        self.create_subscription(Float64MultiArray, 'target/data', self.target_data_callback, 10)
        
        # --- 新增：订阅对面发来的 Yaw Error (用于求和) ---
        # 注意：这个话题由 SerialBridge 创建，用来接收对面发来的误差
        self.create_subscription(Float64, 'target/yaw_error_rx', self.target_yaw_error_rx_callback, 10)

        # ========= 发布 =========
        self.pub_gimbal_cmd = self.create_publisher(Vector3, '/track/gimbal_cmd', 10)
        self.pub_target_angles = self.create_publisher(Vector3, 'target/angles', 10)
        
        # --- 新增：发布新话题 ---
        # 1. 本机的 Yaw Error
        self.pub_tx_yaw_error = self.create_publisher(Float64, 'target/yaw_error_tx', 10)
        # 2. 两机之间的距离
        self.pub_relative_distance = self.create_publisher(Float64, 'target/distance', 10)
        # 3. 误差和
        self.pub_error_sum = self.create_publisher(Float64, 'target/yaw_error_sum', 10)

        # --- 新增：带 Header.stamp 的版本 (方便数据对齐) ---
        self.pub_tx_yaw_error_stamped = self.create_publisher(Vector3Stamped, 'target/yaw_error_tx_stamped', 10)
        self.pub_distance_stamped = self.create_publisher(Vector3Stamped, 'target/distance_stamped', 10)
        self.pub_error_sum_stamped = self.create_publisher(Vector3Stamped, 'target/yaw_error_sum_stamped', 10)

        # ========= 状态量 =========
        self.current_gps = None
        self.current_heading_deg = None
        self.imu_pitch = None

        self.target_gps = None
        self.target_pitch_deg = None
        self.target_heading_deg = None
        
        # --- 新增：存储对面的误差 ---
        self.received_yaw_error = None

        # ========= 控制参数 =========
        # 先用 P 控制，别急着上 PID
        self.declare_parameter('yaw_kp', 0.2)
        self.declare_parameter('pitch_kp', 0.2)

        self.declare_parameter('yaw_vel_limit', 15)
        self.declare_parameter('pitch_vel_limit', 10)

        self.declare_parameter('yaw_deadband_deg', 0.2)
        self.declare_parameter('pitch_deadband_deg', 0.2)

        # 为了防止命令太小带不动，给个最小启动速度
        self.declare_parameter('yaw_min_vel', 0.20)
        self.declare_parameter('pitch_min_vel', 0.15)

        # 方向反了时，只改这个参数即可
        self.declare_parameter('yaw_dir', -1.0)
        self.declare_parameter('pitch_dir', 1.0)

        # 这里按你的要求，先去掉 180° 补偿
        self.declare_parameter('heading_offset_deg', 0.0)

        # 目标俯仰角限幅
        self.declare_parameter('target_pitch_limit_deg', 30.0)

        self.yaw_kp = float(self.get_parameter('yaw_kp').value)
        self.pitch_kp = float(self.get_parameter('pitch_kp').value)

        self.yaw_vel_limit = float(self.get_parameter('yaw_vel_limit').value)
        self.pitch_vel_limit = float(self.get_parameter('pitch_vel_limit').value)

        self.yaw_deadband_deg = float(self.get_parameter('yaw_deadband_deg').value)
        self.pitch_deadband_deg = float(self.get_parameter('pitch_deadband_deg').value)

        self.yaw_min_vel = float(self.get_parameter('yaw_min_vel').value)
        self.pitch_min_vel = float(self.get_parameter('pitch_min_vel').value)

        self.yaw_dir = float(self.get_parameter('yaw_dir').value)
        self.pitch_dir = float(self.get_parameter('pitch_dir').value)

        self.heading_offset_deg = float(self.get_parameter('heading_offset_deg').value)
        self.target_pitch_limit_deg = float(self.get_parameter('target_pitch_limit_deg').value)

        # ========= 日志计时 =========
        self.last_print_time = self.get_clock().now()

        # ========= 控制定时器 =========
        self.create_timer(0.02, self.run_control)

        self.get_logger().info("🧠 数据处理 / 控制节点已启动（P控制，已去掉180°补偿）")

    # ========= 回调 =========
    def local_pitch_callback(self, msg: Float64):
        self.imu_pitch = float(msg.data)

    def local_gps_callback(self, msg: NavSatFix):
        self.current_gps = {
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude
        }

    def local_heading_callback(self, msg: Float64):
        # 保持和你原版一致：默认上游 heading 是弧度
        self.current_heading_deg = (math.degrees(msg.data) + 360.0) % 360.0

    def target_data_callback(self, msg: Float64MultiArray):
        if len(msg.data) != 5:
            self.get_logger().warn(f"target/data 长度错误: {len(msg.data)}")
            return

        self.target_pitch_deg = float(msg.data[0])
        self.target_heading_deg = float(msg.data[1])

        self.target_gps = {
            'lat': float(msg.data[2]),
            'lon': float(msg.data[3]),
            'alt': float(msg.data[4]),
        }
        
    # --- 新增：接收对面发来的误差 ---
    def target_yaw_error_rx_callback(self, msg: Float64):
        self.received_yaw_error = msg.data

    # ========= 控制工具 =========
    def _make_stamped(self, x=0.0, y=0.0, z=0.0):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vector.x = x
        msg.vector.y = y
        msg.vector.z = z
        return msg

    def clamp(self, value, limit_abs):
        return max(min(value, limit_abs), -limit_abs)

    def calc_p_output(self, error, kp, deadband, min_vel, vel_limit):
        """
        纯 P 控制 + 死区 + 最小启动速度 + 限幅
        """
        if abs(error) < deadband:
            return 0.0

        vel = kp * error
        vel = self.clamp(vel, vel_limit)

        if 0.0 < abs(vel) < min_vel:
            vel = math.copysign(min_vel, vel)

        return vel

    # ========= 控制逻辑 =========
    def run_control(self):
        # --- 1. 原有逻辑：获取数据 ---
        if self.current_gps is None:
            return
        if self.current_heading_deg is None:
            return
        if self.imu_pitch is None:
            return
        if self.target_gps is None:
            return

        # --- 2. 原有逻辑：计算控制量 (完全未改动) ---
        # 计算目标方位角 / 目标俯仰角
        bearing = get_bearing_point_2_point_NED(self.current_gps, self.target_gps)
        target_pitch = get_pitch_point_2_point_NED(self.current_gps, self.target_gps)
        target_pitch = max(min(target_pitch, self.target_pitch_limit_deg), -self.target_pitch_limit_deg)

        # 去掉180°补偿后，当前朝向直接就是 heading
        antenna_heading = (self.current_heading_deg + self.heading_offset_deg + 360.0) % 360.0

        # yaw 自动走最短方向
        yaw_error = wrap_angle(bearing - antenna_heading)

        # pitch 误差
        pitch_error = target_pitch - self.imu_pitch

        # 纯P控制
        yaw_vel = self.calc_p_output(
            error=yaw_error,
            kp=self.yaw_kp,
            deadband=self.yaw_deadband_deg,
            min_vel=self.yaw_min_vel,
            vel_limit=self.yaw_vel_limit
        )

        pitch_vel = self.calc_p_output(
            error=pitch_error,
            kp=self.pitch_kp,
            deadband=self.pitch_deadband_deg,
            min_vel=self.pitch_min_vel,
            vel_limit=self.pitch_vel_limit
        )

        # 方向修正
        yaw_vel *= self.yaw_dir
        pitch_vel *= self.pitch_dir

        # --- 3. 原有逻辑：发布控制命令 (完全未改动) ---
        cmd_msg = Vector3()
        cmd_msg.x = yaw_vel
        cmd_msg.y = pitch_vel
        cmd_msg.z = 2.0
        self.pub_gimbal_cmd.publish(cmd_msg)

        # 发布调试角度
        dbg_msg = Vector3()
        dbg_msg.x = bearing
        dbg_msg.y = target_pitch
        dbg_msg.z = 0.0
        self.pub_target_angles.publish(dbg_msg)

        # --- 4. 新增逻辑：新功能 (加在最后，不影响前面的控制) ---
        
        # 4.1 发布本机 Yaw Error
        tx_err_msg = Float64()
        tx_err_msg.data = yaw_error
        self.pub_tx_yaw_error.publish(tx_err_msg)
        self.pub_tx_yaw_error_stamped.publish(self._make_stamped(x=yaw_error))

        # 4.2 计算并发布距离
        d_lat = math.radians(self.target_gps['lat'] - self.current_gps['lat'])
        d_lon = math.radians(self.target_gps['lon'] - self.current_gps['lon'])
        lat_avg = math.radians((self.target_gps['lat'] + self.current_gps['lat']) / 2.0)

        d_north = d_lat * EARTH_RADIUS
        d_east = d_lon * EARTH_RADIUS * math.cos(lat_avg)
        distance = math.sqrt(d_north**2 + d_east**2)

        dist_msg = Float64()
        dist_msg.data = distance
        self.pub_relative_distance.publish(dist_msg)
        self.pub_distance_stamped.publish(self._make_stamped(x=distance))

        # 4.3 计算误差和
        if self.received_yaw_error is not None:
            error_sum = yaw_error + self.received_yaw_error
            sum_msg = Float64()
            sum_msg.data = error_sum
            self.pub_error_sum.publish(sum_msg)
            self.pub_error_sum_stamped.publish(self._make_stamped(x=error_sum))

        # --- 5. 原有日志 (保持不变) ---
        now = self.get_clock().now()
        if (now - self.last_print_time).nanoseconds * 1e-9 >= 1.0:
            # 这里只打印控制相关的日志，不打印新数据，避免刷屏
            self.get_logger().info(
                f"[控制] target_yaw={bearing:.2f}, target_pitch={target_pitch:.2f} | "
                f"heading={self.current_heading_deg:.2f}, antenna_heading={antenna_heading:.2f} | "
                f"yaw_error={yaw_error:.2f}, pitch_error={pitch_error:.2f} | "
                f"yaw_vel={yaw_vel:.3f}, pitch_vel={pitch_vel:.3f}"
            )
            self.last_print_time = now


def main(args=None):
    rclpy.init(args=args)
    node = DataFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
