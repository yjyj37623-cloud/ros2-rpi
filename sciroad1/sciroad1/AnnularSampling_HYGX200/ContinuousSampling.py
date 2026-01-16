from ContinuousConfig import ContinuousConfig
from utils.sampling.xiaomo_sampling import ContinuousSamplingBase
from utils.sampling.base_sampling import Sample200PanBase

import matplotlib as mpl

mpl.use('TkAgg')


class Continuous200Sampling(ContinuousSamplingBase, Sample200PanBase):
    def __init__(self, args):
        super(Continuous200Sampling, self).__init__(args)


def get_batch(sampling):
    from utils.script import get_lstm_one_surface_data
    get_lstm_one_surface_data(sampling)

if __name__ == '__main__':
    config = ContinuousConfig()
    args = config.getArgs()
    haha = Continuous200Sampling(args)

    haha.goback_continuous()
