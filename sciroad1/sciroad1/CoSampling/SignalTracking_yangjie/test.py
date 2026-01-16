import numpy as np
import matplotlib as mpl
from config import TrackingConfig
from utils.sampling.base_sampling import Sample200and300PanBase
from utils.cmdIO import *
import threading
import time
import os
import pandas as pd

mpl.use('TkAgg')

class test(Sample200and300PanBase):

    def __init__(self, args):
        super(test, self).__init__(args)
        self.data = []
        self.collect_count=0
        self.check_name = [
            'acc',
            'dec',
            'v',
            'max_angle200',
            'max_angle300',
            'delay',
            'stride200',
            'stride300',
            'step_block',
            'show_pic',
            'save_pic',
            'data_type'
        ]

    import numpy as np
    import time

    def scan_and_find_peaks(self):
        """
        扫描发射机和接收机的角度，找出信号值在3D平面中的最大极值点。
        返回这些极值点的角度列表。
        """
        signal_values = []  # 存储信号强度和角度的列表

        # 1. 扫描发射机和接收机的角度
        tx_angles = np.arange(self.start_angle, self.end_angle, self.step_size_scan)
        rx_angles = np.arange(self.start_angle, self.end_angle, self.step_size_scan)

        for tx_angle in tx_angles:
            self.pan200_my_pabs(tx_angle)  # 旋转发射机
            time.sleep(1)
            for rx_angle in rx_angles:
                self.pan300_my_pabs(rx_angle)  # 旋转接收机
                time.sleep(3)  # 等待旋转完成
                signal_strength = float(self.rx.getPower())  # 读取信号强度
                signal_values.append((tx_angle, rx_angle, signal_strength))

        # 转换为 NumPy 数组
        signal_values = np.array(signal_values)

        # 获取所有唯一的角度值
        unique_tx_angles = np.unique(signal_values[:, 0])
        unique_rx_angles = np.unique(signal_values[:, 1])

        # 生成网格
        tx_grid, rx_grid = np.meshgrid(unique_tx_angles, unique_rx_angles)
        signal_grid = np.full(tx_grid.shape, -np.inf)  # 初始化信号矩阵，填充最小值

        # 填充信号强度矩阵
        for tx_angle, rx_angle, strength in signal_values:
            tx_idx = np.argmin(np.abs(unique_tx_angles - tx_angle))
            rx_idx = np.argmin(np.abs(unique_rx_angles - rx_angle))
            signal_grid[rx_idx, tx_idx] = strength  # 这里注意 rx_idx 和 tx_idx 的顺序

        # 2. 查找局部极值（峰顶）
        peaks = []
        for i in range(1, signal_grid.shape[0] - 1):  # 排除边界点
            for j in range(1, signal_grid.shape[1] - 1):  # 排除边界点
                # 获取当前点和周围 8 个点的信号强度
                current_value = signal_grid[i, j]
                neighbors = [
                    signal_grid[i - 1, j], signal_grid[i + 1, j],  # 上下
                    signal_grid[i, j - 1], signal_grid[i, j + 1],  # 左右
                    signal_grid[i - 1, j - 1], signal_grid[i - 1, j + 1],  # 左上，右上
                    signal_grid[i + 1, j - 1], signal_grid[i + 1, j + 1],  # 左下，右下
                ]

                # 修改判断条件：大于等于邻居点
                if all(current_value >= neighbor for neighbor in neighbors):
                    peak_tx_angle = tx_grid[i, j]
                    peak_rx_angle = rx_grid[i, j]
                    peaks.append((peak_tx_angle, peak_rx_angle, current_value))

        # 按信号强度降序排序，并返回前10个极值点
        peaks.sort(key=lambda x: x[2], reverse=True)
        top_peaks = peaks[:10]

        # 输出找到的极值点
        print(f"峰顶点（前10个极值点）：{top_peaks}")
        return top_peaks

    def run(self):
        self.pan200.set_zero()
        self.pan300.set_zero()






            #self.save_file(data_point, fig, cmd_args[
if __name__ == '__main__':
     config = TrackingConfig()
     args = config.getArgs()
     code = test(args)
     #code.collect_data()
     code.run()