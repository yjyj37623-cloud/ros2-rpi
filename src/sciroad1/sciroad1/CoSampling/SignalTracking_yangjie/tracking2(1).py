import numpy as np
import matplotlib as mpl
from config import TrackingConfig
from utils.sampling.base_sampling import Sample200and300PanBase
from utils.cmdIO import *
import threading
import time
import queue

mpl.use('TkAgg')

class SignalTracker(Sample200and300PanBase):

    def __init__(self, args):
        super(SignalTracker, self).__init__(args)
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
        self.start_angle=-10  #定义开始角和结束角
        self.end_angle=10
        self.step_size_scan = 2  # 扫描步长，2°
        self.step_size_track = 0.5  # 极值跟踪步长，0.5°
        self.threshold=-30
        #self.init_pan(acc=5, dec=5, v=5)
        self.top_10_peaks=[]
        self.max_interations=10
        # 用于保存数据的列表
        self.data = []
        #self.is_collecting=True #用一个变量控制是否继续采样
        self.count=1   #用一个变量控制信号低于阈值后对准次数，如果过高，则重新对准
        self.max_count = 1 #用来控制最大对准次数
        self.cmd_args = {}

    #     self.power_record =[]  # 采集数据用于绘制图像
    #     self.angle200_record = []
    #     self.angle300_record = []
    #
    #     #启动数据采集线程
    #     self.data_collection_thread = threading.Thread(target=self.collect_data)
    #     self.data_collection_thread.daemon = True  # 设置为守护线程，程序退出时自动结束
    #     self.data_collection_thread.start()
    #
    # def collect_data(self,cmd_args=None):
    #     data_point = {
    #         'time': None,
    #         'rx_angle':None,
    #         'tx_angle': None,
    #         'power':None
    #     }
    #
    #     if cmd_args is None:
    #
    #       self.cmd_args = self._use_config_dict(self.cmd_args, self.check_name)
    #
    #     while self.is_collecting:
    #         # 每隔0.2s采集一次数据
    #         time.sleep(1)
    #
    #         # 获取角度和功率
    #         angle200 = self.pan200.get_p()  # tx_angle
    #         self.angle200_record.append(float(angle200))
    #
    #         angle300 = self.pan300.get_p()  # rx_angle
    #         self.angle300_record.append(float(angle300))
    #
    #         power = float(self.rx.getPower())  # 获取信号功率
    #         self.power_record.append(float(power))
    #
    #         now = time.time()
    #         current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now))
    #
    #         # 将采集的数据添加到data列表中
    #         data_point = {
    #             'time': current_time,
    #             'rx_angle': angle300,
    #             'tx_angle': angle200,
    #             'power': power
    #         }
    #         self.data.append(data_point)  # 将数据保存到列表中
    #
    #         # 也可以进行数据保存、图像生成等操作
    #         # fig = self.show_pic([angle300], [power], xlabel='tx on %s°' % self.args['max_angle200'], ylabel='dBm',
    #         #                     show_pic=self.args['show_pic'])
    #
    #
    # def save_data(self):
    #     # 这里是数据保存的逻辑，如果需要保存成文件，可以实现保存方法
    #     fig = self.show_pic_3D(self.angle200_record, self.angle300_record, self.power_record, xlabel='tx_angle',
    #                            ylabel='rx_angle', zlabel='dBm',
    #                            show_pic=self.cmd_args['show_pic'])
    #
    #     self.save_file_without_io(self.data, fig, 'tracking of the whole procress', self.cmd_args['save_pic'],
    #                               self.cmd_args['data_type'])
    #     pass


    def pan200_my_pab(self,angle):
        self.pan200.p_abs(angle)
        while True:
            current_position = self.pan200.get_p()
            if current_position==angle:
                break

    def pan300_my_pab(self,angle):
        self.pan300.p_abs(angle)
        while True:
            current_position = self.pan300.get_p()
            if current_position==angle:
                break



    def run(self):
        self.pan200.set_zero()  # 将对准位置置为零角度
        self.pan300.set_zero()
        self.top_10_peaks = self.scan_and_find_peaks()

        print("信号强度前十的发射接收机角度和信号强度:")
        for peak in self.top_10_peaks:
            tx_angle, rx_angle, signal_strength = peak  # 解包元组
            print(f"发射机角度: {tx_angle}°, 接收机角度: {rx_angle}, 信号强度: {signal_strength}")

        # 步骤2: 在信号最强的角度运行极值跟踪
        self.track_peak(self.max_interations)

        # 步骤3: 循环监控信号强度并适时切换到最佳角度
        while True:
            time.sleep(1)

            # 检查当前信号强度是否低于阈值
            current_signal = float(self.rx.getPower())
            #print(f"当前信号强度: {current_signal}")

            if current_signal < self.threshold:
                # 信号强度低于阈值时，重新检测最佳角度并进行切换
                print("信号强度低于阈值，重新检测最佳角度...")
                self.count=self.count+1
                self.switch_to_best_angle(self.top_10_peaks)  # 切换角度

                # 进行精确对准
                self.track_peak(self.max_interations)

                #self.is_collecting = False

            if self.count > self.max_count:
                print("找不到合适路径，请重新对准")
                #self.save_data()
                break

    def detailed_scan_for_test(self):
        """
        扫描发射机和接收机的角度，找出信号值最大的三个角度
        返回最大信号值的角度列表
        """
        signal_values = []  # 存储信号强度和角度的列表

        # 1. 发射机以 2° 步进旋转，接收机每次旋转一圈
        tx_angles = np.arange(self.start_angle, self.end_angle, self.step_size_track)
        # self.pan200.p_abs(-1)  #先将转台转到一边，角度范围左闭右开
        for tx_angle in tx_angles:
            self.pan200_my_pab(tx_angle)  # 旋转发射机
            time.sleep(1)
            rx_angles = np.arange(self.start_angle, self.end_angle, self.step_size_track)  # 接收机旋转一圈
            #    self.pan300.p_abs(-1) #每次发射机改变角度时，接收机先回到原位置
            for rx_angle in rx_angles:
                self.pan300_my_pab(rx_angle)  # 旋转接收机
                time.sleep(1)  # 等待旋转完成
                signal_strength = float(self.rx.getPower())  # 读取信号强度,一定不能忽略数据类型，getpower是返回的str
                signal_values.append((tx_angle, rx_angle, signal_strength))

            # time.sleep(1)  # 等待旋转完成

        # 2. 排序并找出信号值最大的三个点
        signal_values.sort(key=lambda x: x[2], reverse=True)  # 按信号强度降序排序
        self.top_10_peaks = signal_values[:10]  # 选出前 3 个信号最大值的角度和信号强度
        print(f"最大十个信号点：{self.top_10_peaks}")
        return self.top_10_peaks


    def scan_and_find_peaks(self):
        """
        扫描发射机和接收机的角度，找出信号值最大的三个角度
        返回最大信号值的角度列表
        """
        signal_values = []  # 存储信号强度和角度的列表


        # 1. 发射机以 2° 步进旋转，接收机每次旋转一圈
        tx_angles = np.arange(self.start_angle, self.end_angle, self.step_size_scan)
        #self.pan200.p_abs(-1)  #先将转台转到一边，角度范围左闭右开
        for tx_angle in tx_angles:
            self.pan200_my_pab(tx_angle)  # 旋转发射机
            time.sleep(1)
            rx_angles = np.arange(self.start_angle, self.end_angle, self.step_size_scan)  # 接收机旋转一圈
        #    self.pan300.p_abs(-1) #每次发射机改变角度时，接收机先回到原位置
            for rx_angle in rx_angles:
                self.pan300_my_pab(rx_angle)  # 旋转接收机
                time.sleep(1)  # 等待旋转完成
                signal_strength = float(self.rx.getPower())  # 读取信号强度,一定不能忽略数据类型，getpower是返回的str
                signal_values.append((tx_angle, rx_angle, signal_strength))


            #time.sleep(1)  # 等待旋转完成

        # 2. 排序并找出信号值最大的三个点
        signal_values.sort(key=lambda x: x[2], reverse=True)  # 按信号强度降序排序
        top_10_peaks = signal_values[:10]  # 选出前 3 个信号最大值的角度和信号强度
        print(f"精扫最大十个信号点：{top_10_peaks}")
        return top_10_peaks


    def switch_to_best_angle(self, prev_angles):
        """
        如果信号强度小于阈值，重新检测之前存储的 3 个角度的当前信号强度，
        选择信号值最大的角度进行切换。
        """
        signal_strength =float(self.rx.getPower())
        if signal_strength < self.threshold:

            #print("信号强度低于阈值，重新检测 3 个角度的信号强度...")

            updated_angles = []

            # 重新测量存储的 3 个角度的信号强度
            for angle in prev_angles:
                tx_angle, rx_angle, _ = angle
                self.pan200_my_pab(tx_angle)  # 旋转到存储的发射机角度
                self.pan300_my_pab(rx_angle)  # 旋转到存储的接收机角度
                time.sleep(1)  # 等待旋转完成
                new_signal_strength = float(self.rx.getPower())  # 读取当前环境下的信号强度
                updated_angles.append((tx_angle, rx_angle, new_signal_strength))
                print(f"角度 Tx = {tx_angle}°, Rx = {rx_angle}° 的新信号强度: {new_signal_strength}")

            # 选择当前环境下信号最强的角度
            best_angle = max(updated_angles, key=lambda x: x[2])
            print(f"切换到最佳角度：Tx = {best_angle[0]}°, Rx = {best_angle[1]}°，信号强度 = {best_angle[2]}")

            # 转到信号最强的角度
            time.sleep(2)  # 等待旋转完成
            self.pan200_my_pab(best_angle[0])
            self.pan300_my_pab(best_angle[1])
            self.top_10_peaks=list(self.top_10_peaks)
            self.top_10_peaks[0] = list(self.top_10_peaks[0])
            self.top_10_peaks[0][0]=best_angle[0] #更新最大角度信息
            self.top_10_peaks[0][1]=best_angle[1]
            return best_angle[0], best_angle[1]

        return None, None


    def track_peak(self, max_iterations):
        tx = self.top_10_peaks[0][0]  # 选择之前信号最强的角度作为起点
        rx = self.top_10_peaks[0][1]
        self.pan200_my_pab(tx)
        self.pan300_my_pab(rx)
        #pan200_position = self.pan200.get_p()
        #pan300_position = self.pan300.get_p()


        for _ in range(max_iterations):
          current_signal = float(self.rx.getPower())
          candidates = []

          # 计算四个方向的信号强度
          for d_tx, d_rx in [(self.step_size_track, 0), (-self.step_size_track, 0),
                            (0, self.step_size_track), (0, -self.step_size_track)]:
              new_tx = tx + d_tx
              new_rx = rx + d_rx
              self.pan200_my_pab(new_tx)
              self.pan300_my_pab(new_rx)
              time.sleep(1)  # 等待设备调整完成
              signal = float(self.rx.getPower())
              candidates.append((signal, new_tx, new_rx))

          # 选择信号最强的方向
          best_signal, best_tx, best_rx = max(candidates, key=lambda x: x[0])

          # 如果没有比当前更好的点，则停止搜索
          if best_signal <= current_signal:
            print(f"局部最大信号点：Tx = {tx}°, Rx = {rx}°")
            break

          # 只有当信号增加时，才更新当前位置
          if best_signal > current_signal:
            rx,tx=best_tx,best_rx

        print(f"局部最大信号点：Tx = {tx}°, Rx = {rx}°")

# 运行主程序
if __name__ == '__main__':
    config = TrackingConfig()
    args = config.getArgs()
    code = SignalTracker(args)

    code.run()
