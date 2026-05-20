from .base_sampling import SampleBase, OnlyRxSampleBase
from utils.cmdIO import *
import matplotlib.pyplot as plt
import pandas as pd

import random
import signal
import time

# --- 新增引用 ---
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3Stamped

class FreqSamplingBase(SampleBase):
    def __init__(self, args):
        super(FreqSamplingBase, self).__init__(args)

        self.check_name = [
            'power',
            'multi',
            'start_freq',
            'end_freq',
            'stride',
            'delay',
            'show_pic',
            'save_pic',
            'data_type',
            'save_name'
        ]

    def get_series_step_freq(self, cmd_args=None):
        if cmd_args is None:
            cmd_args = {}

        cmd_args = self._use_config_dict(cmd_args, self.check_name)


        if (cmd_args['start_freq'] > cmd_args['end_freq']) or cmd_args['stride'] < 0:
            print("freq step config error")
            return

        note2 = io_get_note(self.args)
        note1 = "multi:%s power:%s start_freq:%f end_freq:%f stride:%f" % (
            self.multi,
            self.power,
            cmd_args['start_freq'],
            cmd_args['end_freq'],
            cmd_args['stride'],
        )

        data = {
            'time': [],
            'use_time': [],
            'freq': [],
            'value': [],
            note1: [],
            note2: []
        }

        start_time = time.time()
        sq = cmd_args['start_freq']
        eq = cmd_args['end_freq']
        ss = cmd_args['stride']

        while 1:
            if sq > eq:
                break

            self.tx.setFreq(str(sq))
            self.rx.setFreq(str(sq))
            time.sleep(cmd_args['delay'])
            val = self.rx.getPower()
            now = time.time()

            print(now, sq, val)

            data['time'].append(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now)))
            data['use_time'].append(now - start_time)
            data['freq'].append(sq)
            data['value'].append(float(val))
            data[note1].append(' ')
            data[note2].append(' ')

            sq += ss

        if self.args.cmd is True:
            print(cmd_args['save_name'])

        if cmd_args['show_pic'] or cmd_args['save_pic']:
            fig = self.show_pic(data['freq'], data['value'], xlabel='GHz', ylabel='dBm',
                                show_pic=cmd_args['show_pic'])

        self.save_file(data, fig, cmd_args['save_pic'], cmd_args['data_type'], cmd_args['save_name'])
        plt.close("all")
        return data


class JustKeepSamplingBase(OnlyRxSampleBase):
    def __init__(self, args, ros_node=None): # 1. 增加 ros_node 参数
        self.sampling_flag = getattr(args, 'sampling_flag', True)
        super(JustKeepSamplingBase, self).__init__(args)

        # 2. 初始化 ROS 发布者
        self.ros_node = ros_node
        if self.ros_node:
            self.publisher_ = self.ros_node.create_publisher(Float64MultiArray, 'sample_data', 10)
            self.publisher_stamped_ = self.ros_node.create_publisher(Vector3Stamped, 'sample_data_stamped', 10)
            self.ros_node.get_logger().info("采样类已连接到 ROS Topic: sample_data + sample_data_stamped")
        else:
            self.publisher_ = None
            self.publisher_stamped_ = None

        self.check_name = [
            'power',
            'multi',
            'freq',
            'delay',
            'show_pic',
            'save_pic',
            'data_type',
            'save_name'
        ]

        self.sig = signal.signal(signal.SIGINT, self.int_handler)
        self.stop = False

    def int_handler(self, sig, frame):
        self.stop = True
        print("停")

    def _publish_data(self, val, freq_val, timestamp):
        if self.publisher_:
            msg = Float64MultiArray()
            msg.data = [float(timestamp), float(freq_val), float(val)]
            self.publisher_.publish(msg)

        if self.publisher_stamped_:
            smsg = Vector3Stamped()
            smsg.header.stamp = self.ros_node.get_clock().now().to_msg()
            smsg.vector.x = float(freq_val)
            smsg.vector.y = float(val)
            smsg.vector.z = 0.0
            self.publisher_stamped_.publish(smsg)

    def get_series_step_freq(self, cmd_args=None):
        if cmd_args is None:
            cmd_args = {}

        cmd_args = self._use_config_dict(cmd_args, self.check_name)

        note2 = io_get_note(self.args)
        note1 = "multi:%s power:%s freq:%s" % (
            self.multi,
            self.power,
            cmd_args['freq'],
        )

        data = {
            'time': [],
            'use_time': [],
            'freq': [],
            'value': [],
            note1: [],
            note2: []
        }

        start_time = time.time()
        freq = cmd_args['freq']
        # self.rx.setFreq(str(freq))

        haha = input("输入文件名：")
        name = './data/' + time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime(time.time())) + '_' + haha

        while 1:
            # self.tx.setFreq(str(freq))
            try:
                if self.stop == True:
                    self.save_file(data, None, cmd_args['save_pic'], cmd_args['data_type'], cmd_args['save_name'])
                    break
                time.sleep(cmd_args['delay'])
                
                # --- 核心读取 ---
                val = self.rx.getPower()
                now = time.time()

                print(now, val)

                # --- 新增：实时发布 Topic (包含时间戳) ---
                self._publish_data(val, freq, now)

                # --- 原有逻辑：保存数据 ---
                data['time'].append(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now)))
                data['use_time'].append(now - start_time)
                data['freq'].append(freq)
                data['value'].append(float(val))
                data[note1].append(' ')
                data[note2].append(' ')

                df = pd.DataFrame(data)
                df.to_excel(name + '.xlsx')


                if self.args.cmd is True:
                    print(cmd_args['save_name'])
            except Exception as e:
                print(str(e))
                break

        if cmd_args['show_pic'] or cmd_args['save_pic']:
            fig = self.show_pic(data['freq'], data['value'], xlabel='GHz', ylabel='dBm',
                                show_pic=cmd_args['show_pic'])

        self.save_file(data, fig, cmd_args['save_pic'], cmd_args['data_type'], cmd_args['save_name'])
        plt.close("all")
        return data


# ================= ROS2 节点入口 =================
import rclpy
from rclpy.node import Node
from KeepSampling_1465.StepConfig import StepConfig


class KeepSamplingNode(Node):
    def __init__(self):
        super().__init__('keepsampling_node')
        config = StepConfig()
        args = config.getArgs()
        self.sampler = JustKeepSamplingBase(args, ros_node=self)

    def run(self):
        self.get_logger().info("开始持续功率采样...")
        try:
            self.sampler.get_series_step_freq()
        except KeyboardInterrupt:
            self.get_logger().info("采样已停止")
        except Exception as e:
            self.get_logger().error(f"采样异常: {e}")


def main(args=None):
    import sys
    # 过滤 ROS2 参数，避免 StepConfig 的 parse_args() 报错
    filtered = [sys.argv[0]]
    for a in sys.argv[1:]:
        if a == '--ros-args':
            break
        filtered.append(a)
    sys.argv = filtered

    rclpy.init(args=args)
    node = KeepSamplingNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
