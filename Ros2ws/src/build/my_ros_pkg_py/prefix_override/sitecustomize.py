import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/ROS_APP/ros_workspace/src/install/my_ros_pkg_py'
