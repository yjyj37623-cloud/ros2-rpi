from utils.config.base_config import BaseConfig


class StepConfig(BaseConfig):
    def __init__(self):
        super().__init__()

        # 开始频率，约定小于结束频率，单位为GHz
        self.parser.add_argument('--start_freq', type=float, default=115)

        # 结束频率，约定大于开始频率，单位为GHz
        self.parser.add_argument('--end_freq', type=float, default=170)

        # 步长，约定为正数，单位为GHz
        self.parser.add_argument('--stride', type=float, default=1)

        # 采样间距，单位秒
        self.parser.add_argument('--delay', type=float, default=1)

        # 单条数据测量结束后是否展示曲线
        self.parser.add_argument('--show_pic', type=bool, default=True)

        # 单条数据测量结束后是否保存曲线
        self.parser.add_argument('--save_pic', type=bool, default=True)

        # 数据保存格式，支持txt、xlsx、csv
        self.parser.add_argument('--data_type', type=str, default='xlsx')

        # 是否为命令行执行模式
        self.parser.add_argument('--cmd', type=bool, default=True)

        # 保存文件名
        self.parser.add_argument('--save_name', type=str, default="")

        self.args = self.parser.parse_args()
