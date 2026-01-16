import copy

from config import CoGlassConfig
from utils.sampling.base_sampling import Sample200and300PanBase
from utils.cmdIO import *

import matplotlib as mpl

mpl.use('TkAgg')

class ScatteringSpectrumSampling(Sample200and300PanBase):
    def __init__(self, args):

        super(ScatteringSpectrumSampling, self).__init__(args)

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

    def get_series_step_rel(self, cmd_args=None):
        if cmd_args is None:
            cmd_args = {}

        cmd_args = self._use_config_dict(cmd_args, self.check_name)

        note = io_get_note(self.args)
        data = {
            'time': [],
            'angle200': [],
            'angle300': [],
            'value': [],
            note: []
        }

        neg200 = 0
        if cmd_args['max_angle200'] < 0:
            neg200 = 1

        max_angle200 = copy.deepcopy(cmd_args['max_angle200'])
        max_angle300 = copy.deepcopy(cmd_args['max_angle300'])

        direction_switch = 1

        while 1:
            angle200 = self.pan200.get_p()

            if (neg200 and cmd_args['max_angle200'] >= 0) or (not neg200 and cmd_args['max_angle200'] <= 0):
                break

            val_record = []
            angle300_record = []

            cmd_args['max_angle300'] = direction_switch * max_angle300
            neg300 = 0
            if cmd_args['max_angle300'] < 0:
                neg300 = 1

            direction_switch = -direction_switch

            while 1:
                val = self.rx.getPower()
                val_record.append(float(val))

                now = time.time()
                angle200 = self.pan200.get_p()
                angle300 = self.pan300.get_p()
                angle300_record.append(angle300)
                if angle200 < 0:
                    angle200 = -angle200
                if angle300 < 0:
                    angle300 = -angle300
                print(now, angle200, angle300, val)

                data['time'].append(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now)))
                data['angle200'].append(angle200)
                data['angle300'].append(angle300)
                data['value'].append(float(val))
                data[note].append(' ')

                if (neg300 and cmd_args['max_angle300'] >= 0) or (not neg300 and cmd_args['max_angle300'] <= 0):
                    break

                if neg300:
                    self.pan300.p_rel(-cmd_args['stride300'])
                else:
                    self.pan300.p_rel(cmd_args['stride300'])

                pos300 = -1
                tmp300 = -2

                while 1:
                    if tmp300 == pos300:
                        break
                    tmp300 = pos300
                    pos300 = self.pan300.get_p()

                    time.sleep(0.1)

                time.sleep(cmd_args['delay'])

                if neg300:
                    cmd_args['max_angle300'] += cmd_args['stride300']
                else:
                    cmd_args['max_angle300'] -= cmd_args['stride300']

            if cmd_args['show_pic'] or cmd_args['save_pic']:
                fig = self.show_pic(angle300_record, val_record, xlabel='tx on %s°' % cmd_args['max_angle200'], ylabel='dBm',
                                    show_pic=cmd_args['show_pic'])
                self.save_file_without_io(data, fig, 'tx-%s' % cmd_args['max_angle200'], cmd_args['save_pic'], cmd_args['data_type'])

            if neg200:
                self.pan200.p_rel(-cmd_args['stride200'])
            else:
                self.pan200.p_rel(cmd_args['stride200'])

            pos200 = -1
            tmp200 = -2
            # 通过轮询确定停止位置
            while 1:
                if tmp200 == pos200:
                    break
                tmp200 = pos200
                pos200 = self.pan200.get_p()

                time.sleep(0.1)

            if neg200:
                cmd_args['max_angle200'] += cmd_args['stride200']
            else:
                cmd_args['max_angle200'] -= cmd_args['stride200']


        self.save_file(data, fig, cmd_args['save_pic'], cmd_args['data_type'])

        return data

if __name__ == '__main__':
    config = CoGlassConfig()
    args = config.getArgs()
    haha = ScatteringSpectrumSampling(args)

    haha.get_series_step_rel()