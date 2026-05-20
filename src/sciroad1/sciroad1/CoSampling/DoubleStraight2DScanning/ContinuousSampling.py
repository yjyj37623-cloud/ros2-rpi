import copy

from ContinuousConfig import ContinuousDoubleStright2DScanningConfig
from utils.sampling.base_sampling import SampleDoubleStraight400Base
from utils.cmdIO import *

import matplotlib as mpl

mpl.use('TkAgg')

class ScatteringSpectrumSampling(SampleDoubleStraight400Base):
    def __init__(self, args):

        super(ScatteringSpectrumSampling, self).__init__(args)

        self.check_name = [
            'acc',
            'dec',
            'v',
            'max_posX',
            'max_posY',
            'sampling_gap_delay',
            'strideX',
            # 'strideY',
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
            'posX': [],
            'posY': [],
            'vY': [],
            'value': [],
            note: []
        }

        negX = 0
        if cmd_args['max_posX'] < 0:
            negX = 1

        max_posX = copy.deepcopy(cmd_args['max_posX'])
        max_posY = copy.deepcopy(cmd_args['max_posY'])

        direction_switch = 1

        self.straight2.set_acc_dec_v(cmd_args['acc'], cmd_args['dec'], cmd_args['v'])

        while 1:
            posX = self.straight1.get_p()

            if (negX and cmd_args['max_posX'] >= 0) or (not negX and cmd_args['max_posX'] <= 0):
                break

            val_record = []
            posY_record = []

            # cmd_args['max_posY'] = direction_switch * max_posY
            negY = 0
            if cmd_args['max_posY'] < 0:
                negY = 1

            direction_switch = -direction_switch

            self.straight2.p_rel(direction_switch * cmd_args['max_posY'])

            tmp_now_pos = -1
            tmp_last_pos = -2

            while 1:
                if tmp_now_pos == tmp_last_pos:
                    break

                tmp_last_pos = tmp_now_pos

                val = self.rx.getPower()
                val_record.append(float(val))

                now = time.time()
                posX = self.straight1.get_p()
                posY = self.straight2.get_p()
                tmp_now_pos = posY
                posY_record.append(posY)
                if posX < 0:
                    posX = -posX
                if posY < 0:
                    posY = -posY
                print(now, posX, posY, val)

                vY = self.straight2.get_v()

                data['time'].append(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now)))
                data['posX'].append(posX)
                data['posY'].append(posY)
                data['vY'].append(vY)
                data['value'].append(float(val))
                data[note].append(' ')

                time.sleep(cmd_args['sampling_gap_delay'])

            if cmd_args['show_pic'] or cmd_args['save_pic']:
                fig = self.show_pic(posY_record, val_record, xlabel='tx on %s°' % cmd_args['max_posX'], ylabel='dBm',
                                    show_pic=cmd_args['show_pic'])
                self.save_file_without_io(data, fig, 'tx-%s' % cmd_args['max_posX'], cmd_args['save_pic'], cmd_args['data_type'])

            if negX:
                self.straight1.p_rel(-cmd_args['strideX'])
            else:
                self.straight1.p_rel(cmd_args['strideX'])

            posX = -1
            tmpX = -2
            # 通过轮询确定停止位置
            while 1:
                if tmpX == posX:
                    break
                tmpX = posX
                posX = self.straight1.get_p()

                time.sleep(0.1)

            if negX:
                cmd_args['max_posX'] += cmd_args['strideX']
            else:
                cmd_args['max_posX'] -= cmd_args['strideX']


        self.save_file(data, fig, cmd_args['save_pic'], cmd_args['data_type'])

        return data

if __name__ == '__main__':
    config = ContinuousDoubleStright2DScanningConfig()
    args = config.getArgs()
    haha = ScatteringSpectrumSampling(args)

    haha.get_series_step_rel()