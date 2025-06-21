import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tejaswi/autonomous_docking_ros2/install/docking_controller'
