#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import time

from utils.cmdIO import TrackingConfig
from utils.sampling.base_sampling import SamplePitchand300RotationBase


class GimbalControlNode(Node):
    """
    转台控制节点
    支持：
    1) 绝对角度控制 (P_ABS)
    2) 相对角度控制 (P_REL)
    3) 角速度控制（内部转为 P_REL）
    """

    MODE_ABS = 0
    MODE_REL = 1
    MODE_VEL = 2

    def __init__(self):
        super().__init__('gimbal_control_node')

        # ========= 初始化转台 =========
        config = TrackingConfig()
        args = config.args
        self.gimbal = SamplePitchand300RotationBase(args)

        self.init_gimbal()

        # ========= ROS 参数 =========
        self.declare_parameter('acc', 2.0)
        self.declare_parameter('dec', 2.0)
        self.declare_parameter('vel', 2.0)

        self.acc = self.get_parameter('acc').value
        self.dec = self.get_parameter('dec').value
        self.vel = self.get_parameter('vel').value

        self.set_motion_param()

        # ========= 控制周期 =========
        self.control_dt = 0.02   # 50 Hz
        self.last_cmd_time = time.time()

        # ========= 订阅 =========
        self.create_subscription(
            Vector3,
            '/track/gimbal_cmd',
            self.cmd_callback,
            10
        )

        self.get_logger().info('🎯 转台控制节点已启动')

    # ------------------------------
    def init_gimbal(self):
        self.get_logger().info('转台回零...')
        self.gimbal.pan300.set_zero()
        self.gimbal.pitch.set_zero()

    def set_motion_param(self):
        self.gimbal.pan300.set_acc_dec_v(self.acc, self.dec, self.vel)
        self.gimbal.pitch.set_acc_dec_v(self.acc, self.dec, self.vel)

    # ------------------------------
    def cmd_callback(self, msg: Vector3):
        """
        msg.x : yaw（角度 or 角速度）
        msg.y : pitch（角度 or 角速度）
        msg.z : mode
            0 - 绝对角度
            1 - 相对角度
            2 - 角速度
        """

        now = time.time()
        if now - self.last_cmd_time < self.control_dt:
            return
        self.last_cmd_time = now

        yaw = msg.x
        pitch = msg.y
        mode = int(msg.z)

        if mode == self.MODE_ABS:
            self.gimbal.pan300.p_abs(yaw)
            self.gimbal.pitch.p_abs(pitch)

        elif mode == self.MODE_REL:
            self.gimbal.pan300.p_rel(yaw)
            self.gimbal.pitch.p_rel(pitch)

        elif mode == self.MODE_VEL:
            # 角速度 → 相对角度
            dyaw = yaw * self.control_dt
            dpitch = pitch * self.control_dt

            self.gimbal.pan300.p_rel(dyaw)
            self.gimbal.pitch.p_rel(dpitch)

        else:
            self.get_logger().warn(f'未知控制模式: {mode}')


def main():
    rclpy.init()
    node = GimbalControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
