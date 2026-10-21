#!/bin/bash
# 只规划不飞（独立 ROS master 11312，不碰仿真器）：plan_test.sh v_max a_max [_a_lim:=.. _j_lim:=.. _runout:=..]
source ~/Desktop/LAB4/env.sh
export ROS_MASTER_URI=http://localhost:11312
L=~/Desktop/LAB4
if ! rosparam list > /dev/null 2>&1; then
  setsid nohup roscore -p 11312 > /tmp/plan_roscore.log 2>&1 < /dev/null &
  for i in $(seq 60); do rosparam list > /dev/null 2>&1 && break; sleep 0.5; done
fi
rosparam set /use_sim_time false
rosparam delete /trajectory_generation_node > /dev/null 2>&1; rosparam delete /trajectory_generation_fast_node > /dev/null 2>&1   # private params persist on the master
export ROSCONSOLE_STDOUT_LINE_BUFFERED=1
# fake odometry at the spawn pose
rostopic pub -r 50 /current_state nav_msgs/Odometry "{header: {frame_id: world}, pose: {pose: {position: {x: 0.40, y: 8.0, z: 0.22}, orientation: {z: -0.9929119, w: 0.1188524}}}}" > /dev/null 2>&1 & FP=$!
rosrun planner_pkg traj_vertices_publisher _simulator_data_directory:=$L/vnav-builds/lab4/lab4_Data > /dev/null 2>&1 & GP=$!
LOG=/tmp/plan_test.log; rm -f $LOG
PKG=${TRAJ_PKG:-trajectory_generation}; stdbuf -oL -eL rosrun $PKG ${PKG}_node _v_max:=$1 _a_max:=$2 _publish_rate:=100 "${@:3}" > $LOG 2>&1 & TP=$!
for i in $(seq 300); do grep -q "segment times" $LOG && break; kill -0 $TP 2>/dev/null || break; sleep 0.2; done
grep -E -m2 "trajectory takes|segment times" $LOG | sed 's/.*\]: //; s/\x1b\[0m//'
grep -m1 -E "Check failed|ERROR" $LOG
kill $FP $GP $TP 2>/dev/null; wait $FP $GP $TP 2>/dev/null
