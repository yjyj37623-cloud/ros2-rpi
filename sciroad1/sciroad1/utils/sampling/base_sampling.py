from utils.Device.Device_ShenZhenHengYu.HY300mm import HY300mm
from utils.Device.Device_HengYangGuangXue.LZP3 import LZP3
from utils.Device.Device_HengYangGuangXue.LW_3 import LW3
from utils.Device.Device_Ceyear.RX2438 import Rx2438
from utils.Device.Device_Ceyear.TX1465 import Tx1465
from utils.Device.Device_HengYangGuangXue.LDY_2_400 import LDY_2_400
from utils.cmdIO import *
from utils.Device.util.Serial import Serial
#from mpl_toolkits.mplot3d import Axes3D

import matplotlib.pyplot as plt
import matplotlib as mpl

import pandas

mpl.use('TkAgg')


class SampleBase:
    def __init__(self, args):
        self.args = args
        self.sampling_flag = args.sampling_flag

        if self.sampling_flag is True:
           self.rx = Rx2438(args)

           self.tx = Tx1465(args)

           io_rx_test(self.args, self.rx)

           self.freq, self.power, self.multi = io_set_rx(self.args)

           self.tx.init(self.freq, self.power, self.multi)

           self.rx.setFreq(self.freq)



    def _use_config_dict(self, cmd_args, check_args):
        '''
        用于保证调用参数优先于配置文件参数

        :param cmd_args:
        :param check_args:
        :return:
        '''

        for i in check_args:
            if i not in cmd_args:
                cmd_args[i] = getattr(self.args, i)

        return cmd_args

    def show_pic(self, x, y, xlabel='angle', ylabel='dBm', show_pic=True):
        # plt.clf()
        fig = plt.figure()
        plt.plot(x, y)
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        if show_pic:
            plt.show()

        return fig

    def show_pic_3D(self, x, y, z, xlabel='tx_angle', ylabel='rx_angle', zlabel='dBm', show_pic=True):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')  # 创建 3D 轴
        ax.plot(x, y, z)  # 绘制 3D 曲线
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.set_zlabel(zlabel)

        if show_pic:
            plt.show()

        return fig

    def get_file_name(self, path):
        # name = './data/' + str(time.time()).split('.')[0] + '-F' + str(self.freq) + '-P' + str(self.power) + path
        if path[0] == '_':
            mid = ''
        else:
            mid = '_'
        name = './data/' + time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime(time.time())) + mid + path
        return name

    def save_file(self, data, fig, save_pic, data_type, path=None):
        # 数据保存
        if path is None or path == "":
            path = io_get_file_name(self.args)
        if path != 'n' and path != 'N':
            # 默认路径为XXXSampling_/data/，命名格式为 时间戳-频率-功率-命令.对应格式
            name = self.get_file_name(path)

            if save_pic:
                fig.savefig(name + '.jpg')

            df = pandas.DataFrame(data)
            while 1:
                if data_type == 'xlsx':
                    df.to_excel(name + '.xlsx')
                    break
                elif data_type == 'csv':
                    df.to_csv(name + '.csv')
                    break
                elif data_type == 'txt':
                    df.to_csv(name + '.txt', sep='\t', index=False, header=None)
                    break
                else:
                    data_type = input("文件格式有问题，仅限于txt,xlsx,csv，请重新选择: ")


    def save_file_without_io(self, data, fig, name, save_pic, data_type):
        mid = '_'
        name = './data/' + time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime(time.time())) + mid + name
        if save_pic:
            fig.savefig(name + '.jpg')
        df = pandas.DataFrame(data)
        while 1:
            if data_type == 'xlsx':
                df.to_excel(name + '.xlsx')
                break
            elif data_type == 'csv':
                df.to_csv(name + '.csv')
                break
            elif data_type == 'txt':
                df.to_csv(name + '.txt', sep='\t', index=False, header=None)
                break
            else:
                data_type = input("文件格式有问题，仅限于txt,xlsx,csv，请重新选择: ")

class OnlyRxSampleBase:
    def __init__(self, args):
        self.args = args
        self.debugPan = args.debugPan

        # if self.debugPan is True:
        self.rx = Rx2438(args)

        io_rx_test(self.args, self.rx)

        self.freq, self.power, self.multi = io_set_rx(self.args)

        self.rx.setFreq(self.freq)

    def _use_config_dict(self, cmd_args, check_args):
        '''
        用于保证调用参数优先于配置文件参数

        :param cmd_args:
        :param check_args:
        :return:
        '''

        for i in check_args:
            if i not in cmd_args:
                cmd_args[i] = getattr(self.args, i)

        return cmd_args

    def show_pic(self, x, y, xlabel='angle', ylabel='dBm', show_pic=True):
        # plt.clf()
        fig = plt.figure()
        plt.plot(x, y)
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        if show_pic:
            plt.show()

        return fig

    def get_file_name(self, path):
        # name = './data/' + str(time.time()).split('.')[0] + '-F' + str(self.freq) + '-P' + str(self.power) + path
        if path[0] == '_':
            mid = ''
        else:
            mid = '_'
        name = './data/' + time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime(time.time())) + mid + path
        return name

    def save_file(self, data, fig, save_pic, data_type, path=None):
        # 数据保存
        if path is None or path == "":
            path = io_get_file_name(self.args)
        if path != 'n' and path != 'N':
            # 默认路径为XXXSampling_/data/，命名格式为 时间戳-频率-功率-命令.对应格式
            name = self.get_file_name(path)

            if save_pic:
                fig.savefig(name + '.jpg')

            df = pandas.DataFrame(data)
            while 1:
                if data_type == 'xlsx':
                    df.to_excel(name + '.xlsx')
                    break
                elif data_type == 'csv':
                    df.to_csv(name + '.csv')
                    break
                elif data_type == 'txt':
                    df.to_csv(name + '.txt', sep='\t', index=False, header=None)
                    break
                else:
                    data_type = input("文件格式有问题，仅限于txt,xlsx,csv，请重新选择: ")

class Sample300PanBase(SampleBase):
    def __init__(self, args):
        super(Sample300PanBase, self).__init__(args)
        self.comm = Serial(args)
        self.pan = HY300mm(args, self.comm, obj=1)
        self.pan.init_without_adc()

    def init_pan(self, acc=5.0, dec=5.0, v=5.0):
        self.pan.init(acc, dec, v)


class Sample300PanOnlyRxBase(OnlyRxSampleBase):
        def __init__(self, args):
            super(Sample300PanOnlyRxBase, self).__init__(args)
            self.comm = Serial(args)
            self.pan = HY300mm(args, self.comm, obj=1)
            self.pan.init_without_adc()

        def init_pan(self, acc=5.0, dec=5.0, v=5.0):
            self.pan.init(acc, dec, v)

class Sample200PanBase(SampleBase):
    def __init__(self, args):
        super(Sample200PanBase, self).__init__(args)
        self.comm = Serial(args)
        self.pan = LZP3(args, self.comm)
        self.pan.init_without_adc()

    def init_pan(self, acc=5.0, dec=5.0, v=5.0):
        self.pan.init(acc, dec, v)

class Sample200PanOnlyRxBase(OnlyRxSampleBase):
    def __init__(self, args):
        super(Sample200PanOnlyRxBase, self).__init__(args)
        self.comm = Serial(args)
        self.pan = LZP3(args, self.comm)
        self.pan.init_without_adc()

    def init_pan(self, acc=5.0, dec=5.0, v=5.0):
        self.pan.init(acc, dec, v)

class SamplePitchBase(SampleBase):
    def __init__(self, args):
        super(SamplePitchBase, self).__init__(args)
        self.comm = Serial(args)
        self.pan = LW3(args, self.comm)
        self.pan.init_without_adc()

    def init_pan(self, acc=1.0, dec=1.0, v=1.0):
        self.pan.init(acc, dec, v)


class Sample200and300PanBase(SampleBase):
    def __init__(self, args):
        super(Sample200and300PanBase, self).__init__(args)
        self.comm = Serial(args)
        self.pan200 = LZP3(args, self.comm, obj=0)
        self.pan300 = HY300mm(args, self.comm, obj=1)
        self.pan200.init_without_adc()
        self.pan300.init_without_adc()

    def init_pan(self, acc=5.0, dec=5.0, v=5.0):
        self.pan200.init(acc, dec, v)
        self.pan300.init(acc, dec, v)

class SamplePitchand200RotationBase(SampleBase):
    def __init__(self, args):
        super(SamplePitchand200RotationBase, self).__init__(args)
        self.comm = Serial(args)
        self.pan200 = LZP3(args, self.comm, obj=0)
        self.pitch = LW3(args, self.comm, obj=4)
        self.pan200.init_without_adc()
        self.pitch.init_without_adc()

    def init_pan(self, acc=5.0, dec=5.0, v=5.0):
        self.pan200.init(acc, dec, v)
        self.pitch.init(acc, dec, v)

class SamplePitchand300RotationBase(SampleBase):
    def __init__(self, args):
        super().__init__(args)
        self.comm = Serial(args)
        self.pan300 = HY300mm(args, self.comm, obj=1)
        self.pitch = LW3(args, self.comm, obj=4)
        self.pan300.init_without_adc()
        self.pitch.init_without_adc()

    def init_pan(self, acc=5.0, dec=5.0, v=5.0):
        self.pan200.init(acc, dec, v)
        self.pitch.init(acc, dec, v)

class SampleSingleStraight400Base(SampleBase):
    def __init__(self, args):
        super(SampleSingleStraight400Base, self).__init__(args)
        self.comm = Serial(args)
        self.pan = LDY_2_400(args, self.comm, obj=2)
        self.pan.init_without_adc()

    def init_pan(self, acc=5.0, dec=5.0, v=5.0):
        self.pan.init(acc, dec, v)

class SampleDoubleStraight400Base(SampleBase):
    def __init__(self, args):
        super(SampleDoubleStraight400Base, self).__init__(args)
        self.comm = Serial(args)
        self.straight1 = LDY_2_400(args, self.comm, obj=2)
        self.straight2 = LDY_2_400(args, self.comm, obj=3)
        self.straight1.init_without_adc()
        self.straight2.init_without_adc()

    def init_straight(self, acc=5.0, dec=5.0, v=5.0):
        self.straight1.init(acc, dec, v)
        self.straight2.init(acc, dec, v)
