#!/bin/bash
# 对照实验：保留 Omega_d 项的控制器变体，最终增益跑圆轨迹，然后恢复最终版
source ~/Desktop/LAB3/env.sh
L=~/Desktop/LAB3; SRC=$L/vnav_ws/src/controller_pkg/src/controller_node.cpp; V=$L/results/variants
cp $V/controller_node_omegad.cpp $SRC
(cd $L/vnav_ws && catkin build controller_pkg --no-status 2>&1 | grep -E "error|warning|Failed") 
$L/scripts/fly_test.sh circle 10 5 30 2.5 40 omegad_circle | tail -12
cp $V/controller_node_final.cpp $SRC
(cd $L/vnav_ws && catkin build controller_pkg --no-status > /dev/null) && echo "restored final controller"
cmp $SRC $V/controller_node_final.cpp && echo "source identical to final"
