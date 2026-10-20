#!/bin/bash
# 用官方 launch 文件试飞（增益取自 traj_tracking.launch）：fly_launch.sh <hover|circle> [秒数] [名字]
source ~/Desktop/LAB3/env.sh
L=~/Desktop/LAB3
mode=$1; dur=${2:-40}; name=${3:-final_$mode}
TP=$L/vnav_ws/src/controller_pkg/src/traj_publisher.cpp
want=0; [ "$mode" = hover ] && want=1
if ! grep -q "^#define STATIC_POSE $want" $TP; then
  sed -i "s/^#define STATIC_POSE [01]/#define STATIC_POSE $want/" $TP
  (cd $L/vnav_ws && catkin build controller_pkg --no-status > /dev/null) || exit 1
fi
warm=10; [ "$mode" = hover ] && warm=5
for try in 1 2; do   # retry once if the simulator crashes during the flight
  $L/scripts/sim_ctl.sh ensure || exit 1
  python3 $L/scripts/reset_drone.py > /dev/null; sleep 2
  roslaunch controller_pkg traj_tracking.launch > $L/results/logs/launch_$name.log 2>&1 & P=$!
  python3 $L/scripts/flight_log.py --mode $mode --duration $dur --warmup $warm --out $L/results/flight/$name; ok=$?
  kill -INT $P; wait $P 2>/dev/null
  python3 $L/scripts/reset_drone.py > /dev/null
  [ $ok = 0 ] && exit 0
  echo "flight $name failed (try $try), restarting simulator" >&2
  $L/scripts/sim_ctl.sh restart >&2
done
exit 1
