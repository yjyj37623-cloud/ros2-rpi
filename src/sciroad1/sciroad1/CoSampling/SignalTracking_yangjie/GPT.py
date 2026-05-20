import numpy as np
import time

class BeamTracking:
    def __init__(self, power_meter, tx_rotator, rx_rotator, threshold=-60):
        self.power_meter = power_meter  # 功率计对象
        self.tx_rotator = tx_rotator  # 发射机转台控制对象
        self.rx_rotator = rx_rotator  # 接收机转台控制对象
        self.threshold = threshold  # 信号强度阈值
        self.step_size_scan = 2  # 扫描步长，2°
        self.step_size_track = 0.1  # 极值跟踪步长，0.1°

    def scan_and_find_peaks(self):
        """
        扫描发射机和接收机的角度，找出信号值最大的三个角度
        返回最大信号值的角度列表
        """
        signal_values = []  # 存储信号强度和角度的列表

        # 1. 发射机以 2° 步进旋转，接收机每次旋转一圈
        tx_angles = np.arange(0, 360, self.step_size_scan)
        for tx_angle in tx_angles:
            self.tx_rotator.p_rel(self.step_size_scan)  # 旋转发射机
            time.sleep(1)  # 等待旋转完成
            rx_angles = np.arange(0, 360, self.step_size_scan)  # 接收机旋转一圈
            for rx_angle in rx_angles:
                self.rx_rotator.p_rel(self.step_size_scan)  # 旋转接收机
                time.sleep(0.5)  # 等待旋转完成
                signal_strength = self.power_meter.getPower()  # 读取信号强度
                signal_values.append((tx_angle, rx_angle, signal_strength))

        # 2. 排序并找出信号值最大的三个点
        signal_values.sort(key=lambda x: x[2], reverse=True)  # 按信号强度降序排序
        top_3_peaks = signal_values[:3]  # 选出前 3 个信号最大值的角度和信号强度
        print(f"最大信号点：{top_3_peaks}")
        return top_3_peaks

    def track_max_signal(self, start_tx, start_rx):
        """
        极值跟踪功能：在给定的初始角度开始，以 0.1° 步长逐步寻找信号强度最大的位置
        """
        current_tx = start_tx
        current_rx = start_rx
        max_signal = self.power_meter.getPower()

        while True:
            # 1. 发射机旋转
            self.tx_rotator.p_rel(self.step_size_track)  # 小步进旋转发射机
            time.sleep(0.5)
            tx_power = self.power_meter.getPower()

            # 2. 如果信号强度增大，则更新当前角度和信号值
            if tx_power > max_signal:
                max_signal = tx_power
                current_tx += self.step_size_track

            # 3. 接收机旋转
            self.rx_rotator.p_rel(self.step_size_track)  # 小步进旋转接收机
            time.sleep(0.5)
            rx_power = self.power_meter.getPower()

            if rx_power > max_signal:
                max_signal = rx_power
                current_rx += self.step_size_track

            # 4. 如果信号变化不再增大，跳出循环
            if abs(tx_power - max_signal) < 0.01 and abs(rx_power - max_signal) < 0.01:
                break

        print(f"找到的最大信号点：Tx = {current_tx}°, Rx = {current_rx}°")
        return current_tx, current_rx

    def switch_to_best_angle(self, prev_angles):
        """
        如果信号强度小于阈值，切换到历史上信号值更大的角度
        """
        signal_strength = self.power_meter.getPower()
        if signal_strength < self.threshold:
            # 比较之前的 3 个角度，选择信号值更大的角度
            best_angle = max(prev_angles, key=lambda x: x[2])
            print(f"信号较小，切换到角度：Tx = {best_angle[0]}°, Rx = {best_angle[1]}°")
            self.tx_rotator.p_abs(best_angle[0])  # 转到之前最强信号的发射机角度
            self.rx_rotator.p_abs(best_angle[1])  # 转到之前最强信号的接收机角度
            return best_angle[0], best_angle[1]
        return None, None

    def beam_tracking(self):
        # 1. 扫描并找出最大信号点
        top_3_peaks = self.scan_and_find_peaks()

        # 2. 开始极值跟踪，从最大信号点开始
        best_tx, best_rx = self.track_max_signal(top_3_peaks[0][0], top_3_peaks[0][1])

        # 3. 如果环境改变，信号低于阈值，切换到之前最大信号点之一
        while True:
            time.sleep(1)
            self.switch_to_best_angle(top_3_peaks)  # 检查是否需要切换角度
            time.sleep(1)
            self.track_max_signal(best_tx, best_rx)  # 继续极值跟踪
