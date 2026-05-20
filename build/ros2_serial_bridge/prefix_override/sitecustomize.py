import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yj/桌面/ros2-rpi/install/ros2_serial_bridge'
