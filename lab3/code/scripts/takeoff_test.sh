#!/bin/bash
# 起飞抖动测试（无窗口）：takeoff_test.sh [秒数] [roslaunch 额外参数...]
source ~/Desktop/LAB3/env.sh
L=~/Desktop/LAB3; dur=${1:-20}; shift
$L/scripts/sim_ctl.sh ensure > /dev/null || exit 1
python3 $L/scripts/reset_drone.py > /dev/null; sleep 2
roslaunch controller_pkg traj_tracking.launch "$@" > $L/results/logs/launch_takeoff.log 2>&1 & P=$!
python3 $L/scripts/takeoff_check.py $dur
kill -INT $P; wait $P 2>/dev/null
python3 $L/scripts/reset_drone.py > /dev/null
