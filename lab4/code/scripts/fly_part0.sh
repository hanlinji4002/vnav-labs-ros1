#!/bin/bash
# Part 0：飞到第一个门并悬停，记录 [秒数]
source ~/Desktop/LAB4/env.sh
L=~/Desktop/LAB4; G=$L/results/logs; F=$L/results/flight; mkdir -p $F
dur=${1:-20}
$L/scripts/sim_ctl.sh ensure || exit 1
python3 $L/scripts/reset_drone.py > /dev/null; sleep 3
roslaunch planner_pkg static_point_test.launch simulator_data_directory:=$L/vnav-builds/lab4/lab4_Data > $G/launch_part0.log 2>&1 & P=$!
python3 $L/scripts/hover_log.py --out $F/part0 --duration $dur
kill -INT $P; wait $P 2>/dev/null
python3 $L/scripts/reset_drone.py > /dev/null
