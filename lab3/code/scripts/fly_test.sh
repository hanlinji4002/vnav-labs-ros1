#!/bin/bash
# 一次试飞：fly_test.sh <hover|circle> <kx> <kv> <kr> <komega> [秒数] [名字]
# 需要仿真器和 bridge 已在运行。hover = STATIC_POSE 1，circle = STATIC_POSE 0
source ~/Desktop/LAB3/env.sh
L=~/Desktop/LAB3
mode=$1; kx=$2; kv=$3; kr=$4; ko=$5; dur=${6:-40}; name=${7:-${mode}_${kx}_${kv}_${kr}_${ko}}
TP=$L/vnav_ws/src/controller_pkg/src/traj_publisher.cpp
want=0; [ "$mode" = hover ] && want=1
if ! grep -q "^#define STATIC_POSE $want" $TP; then
  sed -i "s/^#define STATIC_POSE [01]/#define STATIC_POSE $want/" $TP
  (cd $L/vnav_ws && catkin build controller_pkg --no-status > /dev/null) || exit 1
fi
warm=10; [ "$mode" = hover ] && warm=5
mkdir -p $L/results/flight
for try in 1 2; do   # retry once if the simulator crashes during the flight
  $L/scripts/sim_ctl.sh ensure || exit 1
  python3 $L/scripts/reset_drone.py > /dev/null; sleep 2
  rosparam set /kx $kx; rosparam set /kv $kv; rosparam set /kr $kr; rosparam set /komega $ko
  rosrun controller_pkg traj_publisher > /dev/null 2>&1 & P1=$!
  rosrun controller_pkg controller_node > $L/results/logs/ctrl_$name.log 2>&1 & P2=$!
  python3 $L/scripts/flight_log.py --mode $mode --duration $dur --warmup $warm --out $L/results/flight/$name; ok=$?
  kill -INT $P2 $P1 2>/dev/null; wait $P2 $P1 2>/dev/null
  python3 $L/scripts/reset_drone.py > /dev/null
  [ $ok = 0 ] && exit 0
  echo "flight $name failed (try $try), restarting simulator" >&2
  $L/scripts/sim_ctl.sh restart >&2
done
exit 1
