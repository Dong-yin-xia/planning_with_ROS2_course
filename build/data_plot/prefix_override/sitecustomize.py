import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/x/xiugai/planning_with_ROS2_course/install/data_plot'
