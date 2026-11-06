# 公共环境：独立 ROS master（不影响别的实验）+ VM 桌面显示
L=~/Desktop/LAB5
source $L/env.sh
export DISPLAY=:0
export ROS_MASTER_URI=http://localhost:11315
F=$L/results/figures; G=$L/results/logs; mkdir -p $F $G $L/ros_home
