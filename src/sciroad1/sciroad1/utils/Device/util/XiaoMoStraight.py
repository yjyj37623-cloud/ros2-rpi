import serial
import time

xiaomo_config = {
    "Baud": 115200,
    "Timeout": None,
}


class XiaoMoStraight:
    def __init__(self, args, motor_config, comm, angle_reverse=False):
        '''
        LZP3控制接口，遵循rx323串口协议，控制命令遵循小墨科技MT22控制板命令
        :param args:
        '''

        self.comm = comm
        self.safe = motor_config['safe']
        self.name = motor_config['Name']

        print("Check %s: " % self.name + str(self.check()))

        self.motType = motor_config['motType']
        self.MStep = motor_config['MStep']
        self.driveRatio = motor_config['DriveRatio']
        self.angle_reverse = angle_reverse

        # 角度脉冲数，和细分、步进角、转动比有关
        # self.angle_step = int((360 / self.motType * self.MStep * self.driveRatio) / 360)
        self.mm_step = int((360 / self.motType * self.MStep) / self.driveRatio)

    def init_without_adc(self):
        print("设置当前位置为零点 " + str(self.set_zero()) + ", now: " + str(self.get_p()))

        print("设置定位运动模式 " + str(self.set_mode_p()))

    def init(self, acc=1.0, dec=1.0, v=1.0):
        print("设置当前位置为零点 " + str(self.set_zero()) + ", now: " + str(self.get_p()))

        print("设置定位运动模式 " + str(self.set_mode_p()))

        print(
            "当前电机acc dec v " + str(acc) + ", " + str(dec) + ", " + str(v) + ", " + str(self.set_acc_dec_v(acc, dec, v)))

    def set_mode_p(self):
        '''
        设置定位运动模式
        :return:
        '''
        cmd = "MODE_P " + str(self.obj) + " 0"
        self.comm.send(cmd)

        return self.comm.wait_receive()

    def set_acc_dec_v(self, acc, dec, v):
        '''
        设置三个参数
        :param acc:
        :param dec:
        :param v:
        :return:
        '''
        cmd = "P_ACC_DEC_V " + str(self.obj)
        if self.safe['acc'] >= acc > 0:
            cmd += " " + str(int(acc * self.mm_step))
        else:
            print("[ERROR] acc illegal")
            return "[ERROR] acc illegal"

        if self.safe['dec'] >= dec > 0:
            cmd += " " + str(int(dec * self.mm_step))
        else:
            print("[ERROR] dec illegal")
            return "[ERROR] dec illegal"

        if self.safe['v'] >= v > 0:
            cmd += " " + str(int(v * self.mm_step))
        else:
            print("[ERROR] v illegal")
            return "[ERROR] v illegal"

        self.comm.send(cmd)

        res = self.comm.wait_receive()

        print(cmd, res)

        return res

    def p_rel(self, angle):
        '''
        相对定位运动
        :param angle:
        :return:
        '''
        if self.angle_reverse:
            angle = -angle
        cmd = "P_REL " + str(self.obj) + " " + str(angle * self.mm_step)
        self.comm.send(cmd)
        return self.comm.wait_receive()

    def p_abs(self, angle):
        '''
        绝对定位运动
        :param angle:
        :return:
        '''
        if self.angle_reverse:
            angle = -angle
        cmd = "P_ABS " + str(self.obj) + " " + str(angle * self.mm_step)
        self.comm.send(cmd)
        return self.comm.wait_receive()

    def p_stop(self):
        '''
        停止
        :return:
        '''
        cmd = "P_STOP " + str(self.obj)
        self.comm.send(cmd)
        return self.comm.wait_receive()

    def set_zero(self):
        '''
        设置当前位置为坐标系零点
        :return:
        '''
        cmd = "SET_P " + str(self.obj) + " 0"
        self.comm.send(cmd)
        return self.comm.wait_receive()

    def check(self):
        '''
        握手检查
        :return:
        '''
        self.comm.send("CHECK")
        return self.comm.wait_receive()

    def get_p(self):
        '''
        查询转盘当前位置，角度以最近一次设置零点坐标系为主
        :return:
        '''
        cmd = "GET_P " + str(self.obj)
        self.comm.send(cmd)
        return int(self.comm.wait_receive()) / self.mm_step

    def get_v(self):
        cmd = "GET_V " + str(self.obj)
        self.comm.send(cmd)
        return int(self.comm.wait_receive()) / self.mm_step
