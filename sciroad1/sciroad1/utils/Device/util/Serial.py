import serial

xiaomo_config = {
    "Baud": 115200,
    "Timeout": None,
}

class Serial(object):
    def __init__(self, args):
        self.port = args.port
        self.baud = xiaomo_config['Baud']
        self.timeout = xiaomo_config['Timeout']
        self.comm = serial.Serial(self.port, baudrate=self.baud, timeout=self.timeout)

    def send(self, text):
        '''
        统一发送函数
        :param text:
        :return:
        '''
        # self.comm.write(text.encode())
        # now = time.time()
        cmd = text + '\r\n'
        return self.comm.write(cmd.encode())

    def receive(self):
        '''
        统一接受函数
        :return:
        '''
        return self.comm.readline()[:-2]

    def wait_receive(self):
        '''
        阻塞等待，由于电机是异步控制的，所以阻塞市场很短
        :return:
        '''
        while 1:
            if (self.comm.inWaiting() > 0):
                res = self.comm.readline()[:-2]
                return res

    def close(self):
        '''
        关闭连接
        :return:
        '''
        self.comm.close()
        print(self.comm.name + " closed.")

    def open(self):
        '''
        打开连接
        :return:
        '''
        if self.comm.isOpen():
            print(self.port, "open success")
            return True
        else:
            self.comm.open()
            res = self._receive()
            print(res)
            if self.comm.isOpen():
                print(self.port, "open success")
                return True
            else:
                print("open failed")
                return False