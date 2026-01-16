from StepConfig import StepConfig
from utils.sampling.freq_sampling import FreqSamplingBase

import matplotlib as mpl

mpl.use('TkAgg')

class StepFreqSampling(FreqSamplingBase):
    def __init(self, args):
        super(StepFreqSampling, self).__init(args)


if __name__ == '__main__':
    config = StepConfig()
    args = config.getArgs()
    haha = StepFreqSampling(args)

    haha.get_series_step_freq()