#!/bin/bash
# 录 RViz 里飞一圈的视频 + 截图：record_rviz.sh [秒数]
# 用最终增益（traj_tracking.launch）从重生点起飞，RViz 用 results/rviz/lab3_record.rviz
source ~/Desktop/LAB3/env.sh
L=~/Desktop/LAB3; G=$L/results/logs; V=$L/results/video; mkdir -p $V
dur=${1:-30}
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
TP=$L/vnav_ws/src/controller_pkg/src/traj_publisher.cpp
grep -q "^#define STATIC_POSE 0" $TP || { sed -i "s/^#define STATIC_POSE 1/#define STATIC_POSE 0/" $TP; (cd $L/vnav_ws && catkin build controller_pkg --no-status > /dev/null); }
$L/scripts/sim_ctl.sh ensure || exit 1
python3 $L/scripts/reset_drone.py > /dev/null
rviz -d $L/results/rviz/lab3_record.rviz > $G/rviz.log 2>&1 & RV=$!
for i in $(seq 30); do xwininfo -name "lab3_record.rviz - RViz" > /dev/null 2>&1 && break; sleep 1; done
sleep 5
geo=$(xwininfo -name "lab3_record.rviz - RViz")
x=$(echo "$geo" | awk '/Absolute upper-left X/{print $4}'); y=$(echo "$geo" | awk '/Absolute upper-left Y/{print $4}')
w=$(echo "$geo" | awk '/Width:/{print $2}'); h=$(echo "$geo" | awk '/Height:/{print $2}')
w=$((w/2*2)); h=$((h/2*2))
echo "rviz window ${w}x${h}+${x}+${y}"
ffmpeg -y -loglevel error -f x11grab -draw_mouse 0 -framerate 15 -video_size ${w}x${h} -i :0.0+${x},${y} -t $dur \
  -c:v libx264 -preset ultrafast -crf 23 -pix_fmt yuv420p $V/lab3_rviz_circle.mp4 & FF=$!
sleep 1
roslaunch controller_pkg traj_tracking.launch > $G/launch_record.log 2>&1 & P=$!
python3 $L/scripts/flight_log.py --mode circle --duration $((dur-2)) --warmup 10 --out $L/results/flight/record_circle > $V/record_metrics.txt
wait $FF
ffmpeg -y -loglevel error -f x11grab -draw_mouse 0 -video_size ${w}x${h} -i :0.0+${x},${y} -frames:v 1 $V/rviz_circle.png
kill -INT $P; wait $P 2>/dev/null
python3 $L/scripts/reset_drone.py > /dev/null
kill $RV 2>/dev/null
ls -l $V
