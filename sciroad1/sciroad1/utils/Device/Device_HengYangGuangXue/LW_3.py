import time
from utils.Device.util.XiaoMoRotate import XiaoMoRotate

# 电机转盘基础参数设置
#   MStep为细分数，与电机驱动板有关；
#   DriveRatio为转动比，与转盘机械结构有关；
#   motType为步进角，与电机种类有关；
#   obj为当前电机在控制板中的编号，与控制板接线有关
motor_config = {
    "Name": "LW-3",
    "MStep": 20,
    "DriveRatio": 288,
    "motType": 1.8,
    "obj": 4,
    "safe": {
        "acc": 5,
        "dec": 5,
        "v": 5
    }
}


class LW3(XiaoMoRotate):
    def __init__(self, args, comm, obj=4):
        super(LW3, self).__init__(args, motor_config, comm)
        self.obj = obj



if __name__ == '__main__':
    from AnnularSampling_HYGX200.StepConfig import StepConfig

    # 转盘连接测试
    config = StepConfig()
    haha = LW3(config.getArgs())

    # 打印初始位置
    print("start: " + str(haha.get_p()))

    # 旋转1度
    haha.p_rel(1)
    time.sleep(2)

    # 打印结束位置
    print("end: " + str(haha.get_p()))
