import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/eprikovn/ros_ws_robot_2025/install/py_robot_2025_base'
