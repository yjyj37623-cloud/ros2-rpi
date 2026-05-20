#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import time

from utils.cmdIO import TrackingConfig
from utils.sampling.base_sampling import SamplePitchand300RotationBase


class TestPanNode(Node):
    def __init__(self):
        super().__init__('test_pan_node')
        
        # 初始化转台
        config = TrackingConfig()
        args = config.args
        self.gimbal = SamplePitchand300RotationBase(args)

        # 回零
        self.get_logger().info("⏩ 开始回零...")
        self.gimbal.pan300.set_zero()
        time.sleep(1)

        # 测试：直接转 10 度
        self.get_logger().info("🚀 测试开始：PAN 右转 10 度")
        self.gimbal.pan300.p_rel(10.0)  # 让 pan 走 10°
        
        self.get_logger().info("✅ 测试指令已发送！查看转台是否转动")


def main(args=None):
    rclpy.init(args=args)
    node = TestPanNode()
    
    # 只执行一次测试
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

