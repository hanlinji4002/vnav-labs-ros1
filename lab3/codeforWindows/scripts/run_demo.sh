#!/bin/bash
# 一键演示：仿真器（带窗口）+ RViz + 控制器。run_demo.sh [circle|hover]，Ctrl-C 结束并全部关闭
source ~/LAB3/env.sh
L=~/LAB3; G=$L/results/logs; mkdir -p $G
mode=${1:-circle}
TP=$L/vnav_ws/src/controller_pkg/src/traj_publisher.cpp
want=0; [ "$mode" = hover ] && want=1
if ! grep -q "^#define STATIC_POSE $want" $TP; then
  sed -i "s/^#define STATIC_POSE [01]/#define STATIC_POSE $want/" $TP
  (cd $L/vnav_ws && catkin build controller_pkg --no-status > /dev/null) || exit 1
fi
# fresh simulator so the drone starts on the ground
$L/scripts/sim_ctl.sh restart window || exit 1
rviz -d $L/rviz/lab3_demo.rviz > $G/rviz_demo.log 2>&1 & RV=$!
sleep 5   # let RViz open before take-off
roslaunch controller_pkg traj_tracking.launch > $G/launch_demo.log 2>&1 & P=$!
stop() {
  kill -INT $P 2>/dev/null; sleep 3; kill -TERM $P 2>/dev/null; wait $P 2>/dev/null
  kill $RV 2>/dev/null; $L/scripts/sim_ctl.sh stop; exit 0
}
trap stop INT TERM
echo "== 正在飞（$mode）；Ctrl-C 结束 =="
wait $P
stop
