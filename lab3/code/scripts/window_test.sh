#!/bin/bash
# 和录屏相同条件下的飞行检查（不录屏）：window_test.sh [秒数] [roslaunch 参数...]
source ~/Desktop/LAB3/env.sh
L=~/Desktop/LAB3; G=$L/results/logs; dur=${1:-40}; shift
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
SIM_NICE=10 $L/scripts/sim_ctl.sh restart window > /dev/null || exit 1
sleep 3
nice -n 10 rviz -d $L/results/rviz/lab3_sbs.rviz > $G/rviz_wtest.log 2>&1 & RV=$!
sleep 8
roslaunch controller_pkg traj_tracking.launch "$@" > $G/launch_wtest.log 2>&1 & P=$!
python3 $L/scripts/takeoff_check.py $dur $L/results/flight/window_test_attitude.csv
grep -h "smooth start" $G/launch_wtest.log $L/ros_home/log/latest/traj_publisher*.log 2>/dev/null | tail -1
kill -INT $P; sleep 3; kill -TERM $P 2>/dev/null; wait $P 2>/dev/null
kill $RV 2>/dev/null
$L/scripts/sim_ctl.sh stop > /dev/null
