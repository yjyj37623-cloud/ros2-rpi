from .base_sampling import SampleBase, OnlyRxSampleBase
from utils.cmdIO import *
import matplotlib.pyplot as plt
import copy

import random


class ContinuousSamplingBase(SampleBase):
    def __init__(self, args):
        super().__init__(args)

        self.check_name = ['acc_angle', 'dec_angle', 'stop_angle', 'tot_time', 'sampling_gap', 'show_pic', 'save_pic',
                           'data_type', 'save_name']

    def get_series_continuous_rel(self, cmd_args):
        if cmd_args is None:
            cmd_args = {}

        cmd_args = self._use_config_dict(cmd_args, self.check_name)

        neg = 0
        if cmd_args['acc_angle'] < 0 and cmd_args['dec_angle'] < 0 and cmd_args['stop_angle'] < 0:
            neg = 1
        elif cmd_args['acc_angle'] > 0 and cmd_args['dec_angle'] > 0 and cmd_args['stop_angle'] > 0:
            neg = 0
        else:
            print("angle error")
            return

        v = (abs(cmd_args['dec_angle']) - abs(cmd_args['acc_angle'])) / cmd_args['tot_time']
        acc = v / (2 * abs(cmd_args['acc_angle']) / v)
        dec_len = abs(cmd_args['stop_angle']) - abs(cmd_args['dec_angle'])
        dec = v / (2 * dec_len / v)
        print("adv: %f %f %f" % (acc, dec, v))
        if acc > self.pan.safe['acc'] or dec > self.pan.safe['dec'] or v > self.pan.safe['v']:
            print("adv not safe")
            return

        note2 = io_get_note(self.args)
        note1 = "freq:%s power:%s acc_angle:%f dec_angle:%f stop_angle:%f tot_time:%f sampling_gap:%f acc:%f dec:%f v:%f" % (
            self.freq,
            self.power,
            cmd_args['acc_angle'],
            cmd_args['dec_angle'],
            cmd_args['stop_angle'],
            cmd_args['tot_time'],
            cmd_args['sampling_gap'],
            acc,
            dec,
            v
        )
        data = {
            'time': [],
            'use_time': [],
            'angle': [],
            'v': [],
            'value': [],
            note1: [],
            note2: []
        }

        self.pan.set_acc_dec_v(acc, dec, v)
        self.pan.p_rel(cmd_args['stop_angle'])
        start_time = time.time()

        pos = -1
        tmp = -2
        while 1:
            if tmp == pos:
                break
            tmp = pos
            pos = self.pan.get_p()

            val = self.rx.getPower()
            angle = abs(self.pan.get_p())
            vv = abs(self.pan.get_v())
            now = time.time()

            print(now, angle, vv, val)

            data['time'].append(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now)))
            data['use_time'].append(now - start_time)
            data['angle'].append(angle)
            data['v'].append(vv)
            data['value'].append(float(val))
            data[note1].append(' ')
            data[note2].append(' ')
            time.sleep(cmd_args['sampling_gap'])

        if self.args.cmd is True:
            print(cmd_args['save_name'])

        if cmd_args['show_pic'] or cmd_args['save_pic']:
            fig = self.show_pic(data['angle'], data['value'], xlabel='angle', ylabel='dBm',
                                show_pic=cmd_args['show_pic'])

        self.save_file(data, fig, cmd_args['save_pic'], cmd_args['data_type'], cmd_args['save_name'])
        plt.close("all")
        return data

    def goback_continuous(self, cmd_args=None):
        if cmd_args is None:
            cmd_args = {}

        cmd_args = self._use_config_dict(cmd_args, self.check_name)
        go_args = copy.deepcopy(cmd_args)
        go_args['save_name'] += '_GO'
        self.get_series_continuous_rel(go_args)
        back_args = copy.deepcopy(cmd_args)
        back_args['save_name'] += '_BACK'
        back_args['acc_angle'] = -back_args['acc_angle']
        back_args['dec_angle'] = -back_args['dec_angle']
        back_args['stop_angle'] = -back_args['stop_angle']
        self.get_series_continuous_rel(back_args)

    def goback_goback_continuous_batch(self, cmd_args=None, sample_block=None):
        if cmd_args is None:
            cmd_args = {}
        cmd_args = self._use_config_dict(cmd_args, self.check_name)
        self.args.cmd = False
        for i in range(len(cmd_args['speeds'])):
            if cmd_args['save_name'][i] is None:
                cmd_args['save_name'][i] = '_T' + str(cmd_args['speeds'][i])
            else:
                cmd_args['save_name'][i] = cmd_args['save_name'][i] + "_T" + str(cmd_args['speeds'][i])
            cmd_args['tot_time'] = cmd_args['speeds'][i]
            self.goback_continuous(cmd_args)

            if sample_block is not None and i != len(cmd_args['speeds']) - 1:
                input("请移动表面：")


class StepSamplingBase(SampleBase):
    def __init__(self, args):
        super().__init__(args)

        acc, dec, v = io_set_adv(self.args)

        self.init_pan(float(acc), float(dec), float(v))

        self.check_name = ['max_angle', 'delay', 'stride', 'step_block', 'show_pic', 'save_pic', 'data_type']

    # angle正数为顺时针，负数为逆时针
    def get_series_step_rel(self, cmd_args=None):
        '''
        一步一停测得一组数据

        :param end_a: 最终停止的角度，正数为顺时针，负数为逆时针
        :param delay: 电机每步停止后的休眠时间，用于功率计测量
        :param stride: 步长，以度为单位
        :param block: 每步后是否输入阻塞（暂时废弃）
        :param show: 序列之后是否展示
        :return:
        '''

        if cmd_args is None:
            cmd_args = {}

        cmd_args = self._use_config_dict(cmd_args, self.check_name)

        note = io_get_note(self.args)
        data = {
            'time': [],
            'angle': [],
            'value': [],
            note: []
        }

        neg = 0
        if cmd_args['max_angle'] < 0:
            neg = 1

        while 1:
            if self.debugPan:
                val = random.random()
            else:
                val = self.rx.getPower()

            now = time.time()
            angle = self.pan.get_p()
            if angle < 0:
                angle = -angle
            print(now, angle, val)

            io_block(self.args, cmd_args['step_block'])

            data['time'].append(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now)))
            data['angle'].append(angle)
            data['value'].append(float(val))
            data[note].append(' ')

            # 旋转到目标角跳出循环
            if (neg and cmd_args['max_angle'] >= 0) or (not neg and cmd_args['max_angle'] <= 0):
                break

            # 调用接口类旋转函数
            if neg:
                self.pan.p_rel(-cmd_args['stride'])
            else:
                self.pan.p_rel(cmd_args['stride'])

            pos = -1
            tmp = -2
            # 通过轮询确定停止位置
            while 1:
                if tmp == pos:
                    break
                tmp = pos
                pos = self.pan.get_p()

                time.sleep(0.1)

            time.sleep(cmd_args['delay'])

            if neg:
                cmd_args['max_angle'] += cmd_args['stride']
            else:
                cmd_args['max_angle'] -= cmd_args['stride']

        if cmd_args['show_pic'] or cmd_args['save_pic']:
            fig = self.show_pic(data['angle'], data['value'], xlabel='angle', ylabel='dBm',
                                show_pic=cmd_args['show_pic'])

        self.save_file(data, fig, cmd_args['save_pic'], cmd_args['data_type'])

        return data

    def goback_step(self, cmd_args=None):
        '''
        往返采样函数，是get_series_step_rel的简单封装，一次往返后回到原点，参数与get_series_step_rel保持一致

        :param args:
        :return:
        '''

        if cmd_args is None:
            cmd_args = {}

        cmd_args = self._use_config_dict(cmd_args, self.check_name)

        go_args = copy.deepcopy(cmd_args)
        # go_args['save_name'] += '_GO'
        self.get_series_step_rel(go_args)
        back_args = copy.deepcopy(cmd_args)
        # back_args['save_name'] += '_BACK'
        back_args['max_angle'] = -back_args['max_angle']
        self.get_series_step_rel(back_args)
