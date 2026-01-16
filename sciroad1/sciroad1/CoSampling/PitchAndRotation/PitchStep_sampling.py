import copy

from config import P_R_Config
from utils.sampling.base_sampling import SamplePitchand200RotationBase
from utils.cmdIO import *

import matplotlib as mpl

mpl.use('TkAgg')

class ScatteringSpectrumSampling(SamplePitchand200RotationBase):
    def __init__(self, args):

        super(ScatteringSpectrumSampling, self).__init__(args)

        self.check_name = [
            'acc',
            'dec',
            'v',
            'max_angle_Pitch',
            'max_angle_Rotation',
            'delay',
            'stride_Pitch',
            'stride_Rotation',
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
            'angleRotation': [],
            'anglePitch': [],
            'value': [],
            note: []
        }

        neg_Pitch = 0
        if cmd_args['max_angle_Pitch'] < 0:
            neg_Pitch = 1

        max_angleRotation = copy.deepcopy(cmd_args['max_angle_Rotation'])
        max_anglePitch = copy.deepcopy(cmd_args['max_angle_Pitch'])

        direction_switch = 1

        while 1:
            angle200 = self.pitch.get_p()

            if (neg_Pitch and cmd_args['max_angle_Pitch'] >= 0) or (not neg_Pitch and cmd_args['max_angle_Pitch'] <= 0):
                break

            val_record = []
            angleRotation_record = []

            cmd_args['max_angle_Rotation'] = direction_switch * max_angleRotation
            neg_Rotation = 0
            if cmd_args['max_angle_Rotation'] < 0:
                neg_Rotation = 1

            direction_switch = -direction_switch

            while 1:
                val = self.rx.getPower()
                val_record.append(float(val))

                now = time.time()
                anglePitch = self.pitch.get_p()
                angleRotation = self.pan200.get_p()
                angleRotation_record.append(angleRotation)
                if angleRotation < 0:
                    angleRotation = -angleRotation
                if anglePitch < 0:
                    anglePitch = -anglePitch
                print(now, angleRotation, anglePitch, val)

                data['time'].append(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now)))
                data['angleRotation'].append(angleRotation)
                data['anglePitch'].append(anglePitch)
                data['value'].append(float(val))
                data[note].append(' ')

                if (neg_Rotation and cmd_args['max_angle_Rotation'] >= 0) or (not neg_Rotation and cmd_args['max_angle_Rotation'] <= 0):
                    break

                if neg_Rotation:
                    self.pan200.p_rel(-cmd_args['stride_Rotation'])
                else:
                    self.pan200.p_rel(cmd_args['stride_Rotation'])

                pos300 = -1
                tmp300 = -2

                while 1:
                    if tmp300 == pos300:
                        break
                    tmp300 = pos300
                    pos300 = self.pan200.get_p()

                    time.sleep(0.1)

                time.sleep(cmd_args['delay'])

                if neg_Rotation:
                    cmd_args['max_angle_Rotation'] += cmd_args['stride_Rotation']
                else:
                    cmd_args['max_angle_Rotation'] -= cmd_args['stride_Rotation']

            if cmd_args['show_pic'] or cmd_args['save_pic']:
                fig = self.show_pic(angleRotation_record, val_record, xlabel='tx on %s°' % cmd_args['max_angle_Pitch'], ylabel='dBm',
                                    show_pic=cmd_args['show_pic'])
                self.save_file_without_io(data, fig, 'tx-%s' % cmd_args['max_angle_Pitch'], cmd_args['save_pic'], cmd_args['data_type'])

            if neg_Pitch:
                self.pitch.p_rel(-cmd_args['stride_Pitch'])
            else:
                self.pitch.p_rel(cmd_args['stride_Pitch'])

            pos200 = -1
            tmp200 = -2
            # 通过轮询确定停止位置
            while 1:
                if tmp200 == pos200:
                    break
                tmp200 = pos200
                pos200 = self.pitch.get_p()

                time.sleep(0.1)

            if neg_Pitch:
                cmd_args['max_angle_Pitch'] += cmd_args['stride_Pitch']
            else:
                cmd_args['max_angle_Pitch'] -= cmd_args['stride_Pitch']


        self.save_file(data, fig, cmd_args['save_pic'], cmd_args['data_type'])

        return data

if __name__ == '__main__':
    config = P_R_Config()
    args = config.getArgs()
    haha = ScatteringSpectrumSampling(args)

    haha.get_series_step_rel()