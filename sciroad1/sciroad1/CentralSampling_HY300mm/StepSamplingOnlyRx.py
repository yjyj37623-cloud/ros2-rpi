from StepConfig import StepConfig
from utils.sampling.xiaomo_sampling import StepSamplingBase
from utils.sampling.base_sampling import Sample300PanOnlyRxBase

import matplotlib as mpl

mpl.use('TkAgg')

class Step300Sampling(StepSamplingBase, Sample300PanOnlyRxBase):
    def __init__(self, args):
        super(Step300Sampling, self).__init__(args)

if __name__ == '__main__':
    config = StepConfig()
    args = config.getArgs()
    haha = Step300Sampling(args)

    haha.goback_step()
