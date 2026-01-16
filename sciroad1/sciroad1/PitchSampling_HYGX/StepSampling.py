from StepConfig import StepConfig
from utils.sampling.xiaomo_sampling import StepSamplingBase
from utils.sampling.base_sampling import Sample200PanBase

import matplotlib as mpl

mpl.use('TkAgg')

class Step200Sampling(StepSamplingBase, Sample200PanBase):
    def __init__(self, args):
        super(Step200Sampling, self).__init__(args)

if __name__ == '__main__':
    config = StepConfig()
    args = config.getArgs()
    haha = Step200Sampling(args)

    haha.goback_step()
