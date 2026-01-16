import time
from sciroad1.utils.config.base_config import BaseConfig


class TrackingConfig(BaseConfig):
    def __init__(self):
        super().__init__()

        # 加速加速度，米/秒^2
        self.parser.add_argument('--acc', type=float, default=8)

        # 减速加速度，米/秒^2
        self.parser.add_argument('--dec', type=float, default=6)

        # 最大速度阈值，米/秒，超过15会丢步
        self.parser.add_argument('--v', type=float, default=8)

        # 步内等待时间，从转盘完全停止开始计时，秒
        self.parser.add_argument('--delay', type=float, default=5)

        # 步长，可以是小数
        self.parser.add_argument('--stride200', type=float, default=1)
        self.parser.add_argument('--stride300', type=float, default=2)

        # 从当前位置开始的最远转动角度，一定要小心不要打到东西；正数为顺时针，负数为逆时针
        self.parser.add_argument('--max_angle200', type=float, default=-55)
        self.parser.add_argument('--max_angle300', type=float, default=-110)

        # 单条数据测量结束后是否展示曲线
        self.parser.add_argument('--show_pic', type=bool, default=False)

        # 单条数据测量结束后是否保存曲线
        self.parser.add_argument('--save_pic', type=bool, default=True)

        # 每步测量后是否阻塞等待
        self.parser.add_argument('--step_block', type=bool, default=False)

        # 数据保存格式，支持txt、xlsx、csv
        self.parser.add_argument('--data_type', type=str, default='xlsx')

        # 是否为命令行执行模式
        self.parser.add_argument('--cmd', type=bool, default=True)

        # 是否需要采样
        self.parser.add_argument('--sampling_flag', type=bool, default=False)


        self.args = self.parser.parse_args()

def io_rx_test(args, rx):
    if args.cmd is True:
        print("test rx: " + rx.getPower())


def io_set_adv(args):
    a = args.acc
    d = args.dec
    v = args.v

    qq = input("Config文件中当前电机参数adv为：%f %f %f，若正确请敲回车，若不正确请输入n：" % (a, d, v))
    if qq == 'n' or qq == 'N':
        return input("输入电机参数acc dev v，中间用空格隔开：").split()

    return a, d, v


def io_set_rx(args):

    qq = input("Config文件中配置当前频率为%sGHz，当前功率为%sdBm, 倍频为%s，若正确请敲回车，若不正确请输n：" % (
        args.freq, args.power, args.multi))
    if qq == 'n' or qq == 'N':
        freq, power, multi = input("设置当前的频率、功率和倍频，只写数字，默认单位为GHz和dBm，空格隔开：").split()
    else:
        freq = args.freq
        power = args.power
        multi = args.multi

    return freq, power, multi


def io_get_note(args):
    if args.cmd is True:
        note = input("描述这组数据，这段话将写入数据文件中：")
        return note


def io_block(args, block):
    if args.cmd is True and block is True:
        input("回车继续：")


def io_get_file_name(args):
    if args.cmd is True:
        return input("需要保存请起名，不保存输n:")
