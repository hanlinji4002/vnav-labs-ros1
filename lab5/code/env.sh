# 用法：source ~/Desktop/LAB5/env.sh
source /opt/ros/noetic/setup.bash
[ -f ~/Desktop/LAB5/vnav_ws/devel/setup.bash ] && source ~/Desktop/LAB5/vnav_ws/devel/setup.bash
# ROS 日志也放在桌面
export ROS_HOME=~/Desktop/LAB5/ros_home
# no ROS 1 end-of-life pop-up in rviz/rqt
export DISABLE_ROS1_EOL_WARNINGS=1
