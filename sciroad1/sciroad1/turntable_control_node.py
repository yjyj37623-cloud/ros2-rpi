#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import time

from utils.cmdIO import TrackingConfig
from utils.sampling.base_sampling import SamplePitchand300RotationBase


class GimbalControlNode(Node):
    """
    转台控制节点（角速度模式）
    订阅角速度指令，将速度转为相对角度执行
    """

    MODE_VEL = 2  # 固定角速度模式

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
        self.declare_parameter('vel_limit', 5.0)  # 最大角速度 deg/s

        self.acc = self.get_parameter('acc').value
        self.dec = self.get_parameter('dec').value
        self.vel = self.get_parameter('vel').value
        self.vel_limit = self.get_parameter('vel_limit').value

        self.set_motion_param()

        # ========= 控制周期 =========
        self.control_dt = 0.02   # 50 Hz
        self.last_cmd_time = time.time()

        # ========= 订阅 =========
        self.create_subscription(
            Vector3,
            '/track/gimbal_cmd',  # 与数据处理节点保持一致
            self.cmd_callback,
            10
        )

        self.get_logger().info('🎯 转台控制节点已启动（角速度模式）')

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
        msg.x : yaw角速度 (deg/s)
        msg.y : pitch角速度 (deg/s)
        msg.z : mode，忽略，固定为角速度模式
        """
        now = time.time()
        if now - self.last_cmd_time < self.control_dt:
            return
        self.last_cmd_time = now

        # 限制角速度，避免转台受力过大
        yaw_vel = max(min(msg.x, self.vel_limit), -self.vel_limit)
        pitch_vel = max(min(msg.y, self.vel_limit), -self.vel_limit)

        # 将角速度转换为相对角度
        dyaw = yaw_vel * self.control_dt
        dpitch = pitch_vel * self.control_dt

        # 执行相对角度运动
        self.gimbal.pan300.p_rel(dyaw)
        self.gimbal.pitch.p_rel(dpitch)

        self.get_logger().info_throttle(
            0.5,
            f"[执行] dyaw={dyaw:.2f}°, dpitch={dpitch:.2f}° | 原速度 yaw={yaw_vel:.2f}, pitch={pitch_vel:.2f}"
        )


def main():
    rclpy.init()
    node = GimbalControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
