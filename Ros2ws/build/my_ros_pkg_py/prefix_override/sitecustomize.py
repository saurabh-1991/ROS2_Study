import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/ROS_APP/ros_workspace/install/my_ros_pkg_py'
