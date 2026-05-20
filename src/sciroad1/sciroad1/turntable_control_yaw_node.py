#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

from utils.cmdIO import TrackingConfig
from utils.sampling.base_sampling import SamplePitchand300RotationBase


class TurntableControlYawNode(Node):
    """
    only_yaw 转台控制节点

    订阅 /track/gimbal_cmd:
        msg.x -> yaw 角速度 deg/s
        msg.y -> 忽略
        msg.z -> mode（忽略）

    控制方式：
        dyaw = yaw_vel * control_dt
        调用 pan300.p_rel(dyaw)
    """

    def __init__(self):
        super().__init__('turntable_control_yaw_node')

        # ========= 初始化转台 =========
        config = TrackingConfig()
        args = config.args
        self.gimbal = SamplePitchand300RotationBase(args)

        self.init_gimbal()

        # ========= 参数 =========
        self.declare_parameter('acc', 6.0)
        self.declare_parameter('dec', 6.0)
        self.declare_parameter('vel', 10.0)

        self.declare_parameter('vel_limit_yaw', 10.0)
        self.declare_parameter('control_dt', 0.1)
        self.declare_parameter('min_step_yaw_deg', 0.02)

        self.acc = float(self.get_parameter('acc').value)
        self.dec = float(self.get_parameter('dec').value)
        self.vel = float(self.get_parameter('vel').value)

        self.vel_limit_yaw = float(self.get_parameter('vel_limit_yaw').value)
        self.control_dt = float(self.get_parameter('control_dt').value)
        self.min_step_yaw_deg = float(self.get_parameter('min_step_yaw_deg').value)

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

        self.get_logger().info('🎯 turntable_control_yaw_node 已启动（only_yaw）')

    # ========= 初始化 =========
    def init_gimbal(self):
        self.get_logger().info('转台回零...')
        self.gimbal.pan300.set_zero()
        # pitch 不参与控制，可以不动
        # self.gimbal.pitch.set_zero()

    def set_motion_param(self):
        self.gimbal.pan300.set_acc_dec_v(self.acc, self.dec, self.vel)

    # ========= 工具 =========
    def clamp(self, value, limit_abs):
        return max(min(value, limit_abs), -limit_abs)

    # ========= 控制回调 =========
    def cmd_callback(self, msg: Vector3):
        now = time.time()

        # 控制节拍
        if now - self.last_cmd_time < self.control_dt:
            return
        self.last_cmd_time = now

        # ========= 只处理 yaw =========
        yaw_vel = self.clamp(float(msg.x), self.vel_limit_yaw)

        dyaw = yaw_vel * self.control_dt

        # ========= 执行 =========
        if abs(dyaw) >= self.min_step_yaw_deg:
            self.gimbal.pan300.p_rel(dyaw)

        # ========= 日志 =========
        if now - self.last_print_time >= self.print_interval:
            self.get_logger().info(
                f"[执行] dyaw={dyaw:.3f}° | yaw_vel={yaw_vel:.3f}"
            )
            self.last_print_time = now


def main(args=None):
    rclpy.init(args=args)
    node = TurntableControlYawNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
