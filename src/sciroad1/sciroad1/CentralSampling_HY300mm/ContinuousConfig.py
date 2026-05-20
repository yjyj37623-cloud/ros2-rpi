from utils.config.base_config import BaseConfig


class ContinuousConfig(BaseConfig):
    def __init__(self):
        super().__init__()

        # 当前使用的频率，字符串形式，用于记录
        self.parser.add_argument('--freq', type=str, default='140')

        # 当前使用的功率，浮点数形式，用于记录，单位为dbm
        self.parser.add_argument('--power', type=str, default='0')
        #
        # # 加速加速度，米/秒^2
        # self.parser.add_argument('--acc', type=float, default=8)
        #
        # # 减速加速度，米/秒^2
        # self.parser.add_argument('--dec', type=float, default=6)
        #
        # # 最大速度阈值，米/秒，超过15会丢步
        # self.parser.add_argument('--v', type=float, default=8)

        # 加速区长度
        self.parser.add_argument('--acc_angle', type=float, default=-10)

        # 减速角，也就是加速区长度+匀速采样区长度
        self.parser.add_argument('--dec_angle', type=float, default=-110)

        # 停止角，也就是三区总和
        self.parser.add_argument('--stop_angle', type=float, default=-120)

        # 匀速区总用时，单位秒
        self.parser.add_argument('--tot_time', type=float, default=30)

        # 采样间距，单位秒
        self.parser.add_argument('--sampling_gap', type=float, default=0.1)

        # 单条数据测量结束后是否展示曲线
        self.parser.add_argument('--show_pic', type=bool, default=True)

        # 单条数据测量结束后是否保存曲线
        self.parser.add_argument('--save_pic', type=bool, default=True)

        # 数据保存格式，支持txt、xlsx、csv
        self.parser.add_argument('--data_type', type=str, default='xlsx')

        # 是否为命令行执行模式
        self.parser.add_argument('--cmd', type=bool, default=True)

        # 保存文件名
        self.parser.add_argument('--save_name', type=str, default=None)

        self.args = self.parser.parse_args()
