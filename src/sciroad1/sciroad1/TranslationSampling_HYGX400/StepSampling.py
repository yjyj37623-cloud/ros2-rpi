from StepConfig import StepConfig
from utils.sampling.xiaomo_sampling import StepSamplingBase
from utils.sampling.base_sampling import SampleSingleStraight400Base

import matplotlib as mpl

mpl.use('TkAgg')

class StraightStepSampling(StepSamplingBase, SampleSingleStraight400Base):
    def __init__(self, args):
        super(StraightStepSampling, self).__init__(args)

if __name__ == '__main__':
    config = StepConfig()
    args = config.getArgs()
    haha = StraightStepSampling(args)

    haha.goback_step()
