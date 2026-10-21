#!/bin/bash
# 离线规划并导出整条期望轨迹（独立 ROS master 11313，假时钟 10 倍速，不碰仿真器）
# 用法：plan_path.sh <输出前缀> v_max a_max [via_points，如 "[9, -46.5, 5.0, 2.51]"]；默认用副本包 trajectory_generation_via
source ~/Desktop/LAB4/env.sh
export ROS_MASTER_URI=http://localhost:11313
L=~/Desktop/LAB4; out=$1; vmax=$2; amax=$3; via=${4:-[]}
rosparam list > /dev/null 2>&1 || { setsid nohup roscore -p 11313 > /dev/null 2>&1 < /dev/null & for i in $(seq 40); do rosparam list > /dev/null 2>&1 && break; sleep 0.5; done; }
rosparam set /use_sim_time true
rosparam delete /trajectory_generation_node > /dev/null 2>&1
rosparam set /trajectory_generation_node/via_points "$via"
python3 $L/scripts/fake_clock.py 10 > /dev/null 2>&1 & CK=$!
rostopic pub -r 50 /current_state nav_msgs/Odometry "{header: {frame_id: world}, pose: {pose: {position: {x: 0.40, y: 8.0, z: 0.22}, orientation: {z: -0.9929119, w: 0.1188524}}}}" > /dev/null 2>&1 & FP=$!
rosrun planner_pkg traj_vertices_publisher _simulator_data_directory:=$L/vnav-builds/lab4/lab4_Data > /dev/null 2>&1 & GP=$!
python3 $L/scripts/plan_record.py $out > ${out}_record.log 2>&1 & RP=$!
rosrun ${PKG:-trajectory_generation_via} ${NODE:-trajectory_generation_via_node} _v_max:=$vmax _a_max:=$amax _publish_rate:=50 > ${out}_node.log 2>&1 & TP=$!
wait $RP
kill $FP $GP $TP $CK 2>/dev/null; wait $FP $GP $TP $CK 2>/dev/null
grep -m1 "trajectory takes" ${out}_node.log | sed 's/.*\]: //; s/\x1b\[0m//'
cat ${out}_record.log
