# 用法：source ~/Desktop/LAB3/env.sh
source /opt/ros/noetic/setup.bash
[ -f ~/Desktop/LAB3/vnav_ws/devel/setup.bash ] && source ~/Desktop/LAB3/vnav_ws/devel/setup.bash
# tesse-interface 不用 pip，直接加到 PYTHONPATH
export PYTHONPATH=~/Desktop/LAB3/vnav_ws/src/tesse-ros-bridge/tesse-interface/src:$PYTHONPATH
# ROS 日志也放在桌面
export ROS_HOME=~/Desktop/LAB3/ros_home
# no ROS 1 end-of-life pop-up in rviz/rqt
export DISABLE_ROS1_EOL_WARNINGS=1
