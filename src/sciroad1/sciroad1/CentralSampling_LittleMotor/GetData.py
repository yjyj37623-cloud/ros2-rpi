from utils.Device.Device_LittleMotor import LittleMotor
from utils.Device.Device_Ceyear.RX2438 import Rx2438
from config import Config
from utils.cmdIO import *

import threading
import time
import matplotlib.pyplot as plt
import matplotlib as mpl

mpl.use('TkAgg')


class SurfaceData:
    def __init__(self, args):
        self.args = args
        self.rx = Rx2438(self.args)

        io_rx_test(self.args, self.rx)

        self.arduino = LittleMotor(args)
        # self.file_path = self.args.DataPath

        self.freq, self.power = io_set_rx(self.args)

    def getOne(self, start_angle, end_angle, speed, dirr=None):
        # TODO:
        #  1.改为异步通信，去掉多线层阻塞
        #  2.增加睡眠控制

        if dirr is None:
            dirr = self.args.default_dir
        # start
        self.arduino.Round(dirr, start_angle, self.args.default_speed)

        tnum1 = len(threading.enumerate())
        # get data
        seqs = []
        thread_round = threading.Thread(target=self.arduino.Round, args=(dirr, end_angle - start_angle, speed))
        thread_round.start()
        start = time.time()
        while len(threading.enumerate()) != tnum1:
            val = self.rx.getPower()
            now = time.time()
            seqs.append([val, now])
            time.sleep(speed * 2.0 / 1000000)

        print(len(seqs))

        # end
        if dirr == 'CW':
            self.arduino.Round('CCW', end_angle, self.args.default_speed)
        else:
            self.arduino.Round('CW', end_angle, self.args.default_speed)

        return seqs, start

    def get_series_step_rel(self, start_angle, end_angle, speed, dirr):
        pass

    def _append_file(self, data):
        data.to_csv(self.file_path, mode='a', index=False, header=False)
        return

    def getBatch(self):
        pass


if __name__ == '__main__':
    config = Config()
    args = config.getArgs()
    data = SurfaceData(args)

    data, start = data.getOne(45, 150, 8000)
    print(data)
    x = []
    y = []
    for i in data:
        x.append(i[1] - start)
        y.append(float(i[0]))
    print(x)
    print(y)
    # for i in range(len(x)):
    plt.plot(x, y)

    plt.xlabel("t")
    plt.ylabel("dBm")
    # plt.grid()
    plt.show()
