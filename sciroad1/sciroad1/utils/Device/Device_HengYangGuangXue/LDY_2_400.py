import time
from utils.Device.util.XiaoMoStraight import XiaoMoStraight

motor_config = {
    "Name": "LDY-400",
    "MStep": 20,
    "DriveRatio": 4,
    "motType": 1.8,
    "obj": 3,
    "safe": {
        "acc": 5,
        "dec": 5,
        "v": 5
    }
}

class LDY_2_400(XiaoMoStraight):
    def __init__(self, args, comm, obj=2):
        super().__init__(args, motor_config, comm)
        self.obj = obj

if __name__ == '__main__':
    from CoSampling.DoubleStraight2DScanning.StepConfig import StepDoubleStright2DScanningConfig
    from utils.Device.util.Serial import Serial

    # 转盘连接测试
    config = StepDoubleStright2DScanningConfig()
    comm = Serial(config)
    haha = LDY_2_400(config.getArgs())

    # 打印初始位置
    print("start: " + str(haha.get_p()))

    # 旋转1度
    haha.p_rel(1)
    time.sleep(2)

    # 打印结束位置
    print("end: " + str(haha.get_p()))