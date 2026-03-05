import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mnt/Volume/Github/DroneDockingSim/ros2_ws/install/drone_docking_sim'
