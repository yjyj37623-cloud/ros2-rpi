#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Vector3
from tf_transformations import euler_from_quaternion

EARTH_RADIUS = 6371000.0  # m


# ================= 工具函数 =================
def get_bearing_point_2_point_NED(current, target):
    lat1, lon1 = math.radians(current['lat']), math.radians(current['lon'])
    lat2, lon2 = math.radians(target['lat']), math.radians(target['lon'])
    dlon = lon2 - lon1
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dlon)
    brng = math.atan2(y, x)
    return (math.degrees(brng) + 360) % 360


def get_pitch_point_2_point_NED(current, target):
    dlat = math.radians(target['lat'] - current['lat'])
    dlon = math.radians(target['lon'] - current['lon'])
    d_north = dlat * EARTH_RADIUS
    d_east = dlon * EARTH_RADIUS * math.cos(math.radians(current['lat']))
    d_alt = target['alt'] - current['alt']
    horizontal = math.sqrt(d_north**2 + d_east**2)
    return math.degrees(math.atan2(d_alt, horizontal))


def wrap_angle(angle):
    while angle >= 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


# ================= PID 类 =================
class PID:
    def __init__(self, kp, ki, kd, limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.limit = limit
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return max(min(out, self.limit), -self.limit)


# ================= 主节点 =================
class DataFusionNode(Node):
    def __init__(self):
        super().__init__('data_fusion_node')

        # 订阅
        self.create_subscription(Imu, 'handsfree/imu', self.imu_callback, 10)
        self.create_subscription(NavSatFix, 'gps/fix', self.gps_callback, 10)
        self.create_subscription(NavSatFix, 'target/gps', self.target_gps_callback, 10)

        # 发布
        self.pub_target_angles = self.create_publisher(Vector3, 'target/angles', 10)
        self.pub_pid_output = self.create_publisher(Vector3, 'pid/output', 10)

        # 状态量
        self.current_gps = None
        self.target_gps = None
        self.imu_yaw = 0.0
        self.imu_pitch = 0.0
        self.imu_wz = 0.0
        self.imu_wy = 0.0

        # PID
        self.outer_pid_yaw = PID(0.05, 0.0, 0.01, 0.5)
        self.outer_pid_pitch = PID(0.05, 0.0, 0.01, 0.5)
        self.inner_pid_yaw = PID(0.08, 0.0, 0.02, 0.75)
        self.inner_pid_pitch = PID(0.08, 0.0, 0.02, 0.75)

        self.last_time = self.get_clock().now()
        self.create_timer(0.02, self.run_control)  # 50 Hz

        self.get_logger().info("🧠 数据处理 / 控制节点已启动")

    # ========= 回调 =========
    def imu_callback(self, msg):
        q = msg.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.imu_pitch = math.degrees(pitch)
        self.imu_yaw = (math.degrees(yaw) + 360) % 360
        self.imu_wy = msg.angular_velocity.y
        self.imu_wz = msg.angular_velocity.z

    def gps_callback(self, msg):
        self.current_gps = {'lat': msg.latitude, 'lon': msg.longitude, 'alt': msg.altitude}

    def target_gps_callback(self, msg):
        self.target_gps = {'lat': msg.latitude, 'lon': msg.longitude, 'alt': msg.altitude}

    # ========= 控制逻辑 =========
    def run_control(self):
        if self.current_gps is None or self.target_gps is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0:
            return
        self.last_time = now

        bearing = get_bearing_point_2_point_NED(self.current_gps, self.target_gps)
        pitch = get_pitch_point_2_point_NED(self.current_gps, self.target_gps)

        yaw_error = wrap_angle(bearing - self.imu_yaw)
        pitch_error = pitch - self.imu_pitch

        yaw_rate_ref = self.outer_pid_yaw.compute(yaw_error, dt)
        pitch_rate_ref = self.outer_pid_pitch.compute(pitch_error, dt)

        yaw_out = self.inner_pid_yaw.compute(yaw_rate_ref - self.imu_wz, dt)
        pitch_out = self.inner_pid_pitch.compute(pitch_rate_ref - self.imu_wy, dt)

        target_msg = Vector3(x=bearing, y=pitch, z=0.0)
        self.pub_target_angles.publish(target_msg)

        pid_msg = Vector3(x=yaw_out, y=pitch_out, z=0.0)
        self.pub_pid_output.publish(pid_msg)

        self.get_logger().info_throttle(
            1.0,
            f"[控制] 目标(yaw={bearing:.2f}, pitch={pitch:.2f}) | 输出(yaw={yaw_out:.3f}, pitch={pitch_out:.3f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = DataFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

