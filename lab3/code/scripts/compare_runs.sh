#!/bin/bash
# 对比实验：指令频率 vs 推力（开环）；仿真时间定时器 vs 墙钟定时器（闭环，增益 6/4/16/2.5）
source ~/Desktop/LAB3/env.sh
L=~/Desktop/LAB3; R=$L/results; SRC=$L/vnav_ws/src/controller_pkg/src/controller_node.cpp
$L/scripts/sim_ctl.sh ensure || exit 1
# 1. open loop: equal prop speeds 55 rad/s sent at different rates
{ echo "# open loop, 4 x 55 rad/s for 1.5 s; model: m=1, cf=1e-3 -> hover thrust 9.81 N"
  for r in 20 50 100 200 500 1000; do printf "rate %4s Hz: " $r; RATE=$r python3 $L/scripts/openloop_thrust.py 55 | tail -1; done
} > $R/openloop_rate.txt
cat $R/openloop_rate.txt
# 2. closed loop with the simulated-time timer (the variant kept in results/variants)
cp $SRC $R/variants/controller_node_wall.cpp
cp $R/variants/controller_node_simtimer.cpp $SRC
(cd $L/vnav_ws && catkin build controller_pkg --no-status > /dev/null) || exit 1
$L/scripts/fly_test.sh hover 6 4 16 2.5 15 hover_simtimer | tail -9
$L/scripts/fly_test.sh circle 6 4 16 2.5 40 circle_simtimer | tail -12
# restore the wall-clock version
cp $R/variants/controller_node_wall.cpp $SRC
(cd $L/vnav_ws && catkin build controller_pkg --no-status > /dev/null) && echo "restored wall-clock controller"
