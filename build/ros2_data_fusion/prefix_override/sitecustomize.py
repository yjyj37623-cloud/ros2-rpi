import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hhhh/Desktop/ros2-rpi/install/ros2_data_fusion'
