import pyvisa as visa
from AnnularSampling_HYGX200.StepConfig import StepConfig


class Tx1465:
    def __init__(self, args):
        self.args = args
        rm = visa.ResourceManager()
        # res = rm.list_resources()
        # print('find resources: ', res)
        self.haha = rm.open_resource(args.txPath, read_termination='\n')
        print("Tx连接成功: " + str(self.haha))
        # self.setFreq(self.args.freq)
        # print("Rx2438 freq :", self.args.freq)

    def check(self):
        print(self._query('*IDN??\n'))

    def init(self, freq, power, multi):
        self.set_mode()
        self.setMultiplier(multi)
        self.setPower(power)
        self.setFreq(freq)
        self.set_on()
        print("set tx: freq %s power %s multi %s, 信号打开" % (freq, power, multi))

    def init_without_freq(self, power, multi):
        self.set_mode()
        self.setMultiplier(multi)
        self.setPower(power)
        self.set_on()
        print("set tx: power %s multi %s, 信号打开" % (power, multi))

    def set_on(self):
        print("射频输出")
        return self._write('OUTPut 1\n')

    def set_off(self):
        print("射频关闭")
        return self._write('OUTPut 0\n')

    def set_mode(self):
        print("设置连续波信号")
        return self._write('FREQ:FIX\n')

    def setMultiplier(self, num):
        print("tx set multiplier: %s" % num)
        return self._write('FREQuency:MULTiplier ' + num + '\n')

    def setPower(self, power):
        print("tx set power: %sdBm" % power)
        return self._write('POWer ' + power + 'dBm\n')

    def setFreq(self, freq):
        print("tx set freq: %sGHz" % freq)
        return self._write('FREQ ' + freq + 'GHz\n')

    def _query(self, cmd):
        return self.haha.query(cmd)

    def _write(self, cmd):
        return self.haha.write(cmd)


if __name__ == '__main__':
    config = StepConfig()
    args = config.getArgs()
    rx = Tx1465(args)
    print(rx.check())
