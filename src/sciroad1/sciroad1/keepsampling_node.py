#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from utils.cmdIO import StepConfig
from utils.sampling.freq_sampling import JustKeepSamplingBase

import matplotlib as mpl
mpl.use('TkAgg')


class StepFreqSampling(JustKeepSamplingBase):
    def __init__(self, args, ros_node): # 1. 接收 ros_node
        self.sampling_flag=True
        # 2. 传给父类
        super(StepFreqSampling, self).__init__(args, ros_node=ros_node)


class KeepSamplingNode(Node):
    def __init__(self):
        super().__init__('keepsampling_node')
        
        self.get_logger().info('📡 keepsampling_node 启动')

        # 初始化采样配置
        config = StepConfig()
        args = config.getArgs()

        # 初始化采样对象
        # 3. 把 'self' 传进去，让采样类可以使用 publish 功能
        self.sampler = StepFreqSampling(args, ros_node=self)

        # 只执行一次，避免在 __init__ 里直接阻塞太久
        self.started = False
        self.timer = self.create_timer(0.5, self.run_once)

    def run_once(self):
        if self.started:
            return

        self.started = True
        self.get_logger().info('🚀 开始执行连续采样任务...')
        self.timer.cancel()

        try:
            self.sampler.get_series_step_freq()
            self.get_logger().info('✅ 采样任务执行完成')
        except Exception as e:
            self.get_logger().error(f'❌ 采样任务执行失败: {e}')
        finally:
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = KeepSamplingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 用户中断，节点退出')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
