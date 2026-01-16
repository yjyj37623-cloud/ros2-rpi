from StepConfig import StepConfig
from utils.sampling.freq_sampling import JustKeepSamplingBase
import signal


import matplotlib as mpl

mpl.use('TkAgg')

# stop = False
# def haha(signum, frame):
#     global stop
#     stop = True
#     print("停止")

# signal.signal(signal.SIGINT,  haha)

class StepFreqSampling(JustKeepSamplingBase):
    def __init(self, args):
        super(StepFreqSampling, self).__init(args)


if __name__ == '__main__':
    config = StepConfig()
    args = config.getArgs()

    haha = StepFreqSampling(args)

    haha.get_series_step_freq()