#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

from utils.cmdIO import TrackingConfig
from utils.sampling.base_sampling import SamplePitchand300RotationBase


class GimbalControlNode(Node):
    """
    转台控制节点
    订阅 /track/gimbal_cmd:
        msg.x -> yaw 角速度 deg/s
        msg.y -> pitch 角速度 deg/s
        msg.z -> mode，当前固定忽略

    实际执行方式：
        角速度 * control_dt -> 小步进相对位移
        然后调用 p_rel() 执行
    """

    MODE_VEL = 2

    def __init__(self):
        super().__init__('gimbal_control_node')

        # ========= 初始化转台 =========
        config = TrackingConfig()
        args = config.args
        self.gimbal = SamplePitchand300RotationBase(args)

        self.init_gimbal()

        # ========= 参数 =========
        self.declare_parameter('acc', 6.0)
        self.declare_parameter('dec', 6.0)
        self.declare_parameter('vel', 10.0)

        # 最终允许的最大控制角速度（deg/s）
        self.declare_parameter('vel_limit_yaw', 10)
        self.declare_parameter('vel_limit_pitch', 5)

        # 控制周期：50Hz
        self.declare_parameter('control_dt', 0.2)

        # 极小位移不执行，减少抖动和异响
        self.declare_parameter('min_step_yaw_deg', 0.02)
        self.declare_parameter('min_step_pitch_deg', 0.1)

        self.acc = float(self.get_parameter('acc').value)
        self.dec = float(self.get_parameter('dec').value)
        self.vel = float(self.get_parameter('vel').value)

        self.vel_limit_yaw = float(self.get_parameter('vel_limit_yaw').value)
        self.vel_limit_pitch = float(self.get_parameter('vel_limit_pitch').value)

        self.control_dt = float(self.get_parameter('control_dt').value)

        self.min_step_yaw_deg = float(self.get_parameter('min_step_yaw_deg').value)
        self.min_step_pitch_deg = float(self.get_parameter('min_step_pitch_deg').value)

        self.set_motion_param()

        # ========= 状态 =========
        self.last_cmd_time = time.time()
        self.last_print_time = time.time()
        self.print_interval = 1.0

        # ========= 订阅 =========
        self.create_subscription(
            Vector3,
            '/track/gimbal_cmd',
            self.cmd_callback,
            10
        )

        self.get_logger().info('🎯 转台控制节点已启动')

    def init_gimbal(self):
        self.get_logger().info('转台回零...')
        self.gimbal.pan300.set_zero()
        self.gimbal.pitch.set_zero()

    def set_motion_param(self):
        self.gimbal.pan300.set_acc_dec_v(self.acc, self.dec, self.vel)
        self.gimbal.pitch.set_acc_dec_v(self.acc, self.dec, self.vel)

    def clamp(self, value, limit_abs):
        return max(min(value, limit_abs), -limit_abs)

    def cmd_callback(self, msg: Vector3):
        now = time.time()

        # 固定节拍执行，避免回调频率过高导致命令堆太快
        if now - self.last_cmd_time < self.control_dt:
            return
        self.last_cmd_time = now

        # ========= 限速 =========
        yaw_vel = self.clamp(float(msg.x), self.vel_limit_yaw)
        pitch_vel = self.clamp(float(msg.y), self.vel_limit_pitch)

        # ========= 速度 -> 小步进角度 =========
        dyaw = yaw_vel * self.control_dt
        dpitch = pitch_vel * self.control_dt

        # ========= 执行 =========
        if abs(dyaw) >= self.min_step_yaw_deg:
            self.gimbal.pan300.p_rel(dyaw)

        if abs(dpitch) >= self.min_step_pitch_deg:
            self.gimbal.pitch.p_rel(dpitch)

        # ========= 日志 =========
        if now - self.last_print_time >= self.print_interval:
            self.get_logger().info(
                f"[执行] dyaw={dyaw:.3f}°, dpitch={dpitch:.3f}° | "
                f"yaw_vel={yaw_vel:.3f}, pitch_vel={pitch_vel:.3f}"
            )
            self.last_print_time = now


def main():
    rclpy.init()
    node = GimbalControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

