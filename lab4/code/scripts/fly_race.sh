#!/bin/bash
# 飞一次竞速并记录：fly_race.sh <名字> [v_max a_max publish_rate] [bag]
# 默认是模板参数 8 / 4 / 5 Hz；第 5 个参数写 bag 会同时录 rosbag；第 6 个是控制频率（默认 1000）
source ~/Desktop/LAB4/env.sh
L=~/Desktop/LAB4; G=$L/results/logs; F=$L/results/flight; mkdir -p $F
name=$1; vmax=${2:-8.0}; amax=${3:-4.0}; rate=${4:-5.0}; chz=${6:-1000}
SIMDATA=$L/vnav-builds/lab4/lab4_Data
for try in 1 2; do   # retry once if the simulator crashes during the flight
  [ -n "$NOENSURE" ] || $L/scripts/sim_ctl.sh ensure || exit 1
  python3 $L/scripts/reset_drone.py > /dev/null; sleep 3
  [ "$5" = bag ] && { rosbag record -O $F/$name.bag /current_state /desired_state > $G/bag_$name.log 2>&1 & BAG=$!; }
  python3 $L/scripts/race_log.py --out $F/$name --duration 150 > /dev/null & REC=$!
  sleep 1
  roslaunch ${TRAJ_PKG:-trajectory_generation} traj_following.launch v_max:=$vmax a_max:=$amax publish_rate:=$rate control_hz:=$chz $LAUNCH_ARGS > $G/launch_$name.log 2>&1 & P1=$!
  sleep 2
  roslaunch planner_pkg traj_gen.launch simulator_data_directory:=$SIMDATA > $G/gen_$name.log 2>&1 & P2=$!
  wait $REC
  kill -INT $P1 $P2 ${BAG:-}; wait $P1 $P2 ${BAG:-} 2>/dev/null
  python3 $L/scripts/reset_drone.py > /dev/null
  cat $F/${name}_summary.txt
  grep -q "race time" $F/${name}_summary.txt && python3 $L/scripts/sim_ok.py 3 && exit 0
  echo "flight $name failed (try $try), restarting simulator" >&2
  $L/scripts/sim_ctl.sh restart >&2
done
exit 1
