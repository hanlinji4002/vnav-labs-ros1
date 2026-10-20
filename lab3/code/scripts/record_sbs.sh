#!/bin/bash
# 并排录屏（像 MIT 的 result.gif）：左 RViz，右 Unity 仿真窗口。record_sbs.sh [秒数]  （会重启仿真器，从起飞开始录）
# 要求：虚拟机桌面已登录并解锁
source ~/Desktop/LAB3/env.sh
L=~/Desktop/LAB3; G=$L/results/logs; V=$L/results/video; mkdir -p $V
dur=${1:-32}
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
TP=$L/vnav_ws/src/controller_pkg/src/traj_publisher.cpp
grep -q "^#define STATIC_POSE 0" $TP || { sed -i "s/^#define STATIC_POSE 1/#define STATIC_POSE 0/" $TP; (cd $L/vnav_ws && catkin build controller_pkg --no-status > /dev/null); }
# fresh simulator with its window (drone on the ground at spawn); Unity rendering at low priority
SIM_NICE=10 $L/scripts/sim_ctl.sh restart window || exit 1
sleep 3
# keep the screen from locking while recording
gnome-session-inhibit --inhibit idle --reason lab3-recording sleep $((dur + 60)) > /dev/null 2>&1 & INH=$!
# park the pointer in the bottom-right corner so no dock tooltip covers the capture
python3 -c "import ctypes; X=ctypes.cdll.LoadLibrary('libX11.so.6'); X.XOpenDisplay.restype=ctypes.c_void_p; d=ctypes.c_void_p(X.XOpenDisplay(None)); X.XWarpPointer(d,0,ctypes.c_ulong(X.XDefaultRootWindow(d)),0,0,0,0,1275,795); X.XFlush(d)"
nice -n 10 rviz -d $L/results/rviz/lab3_sbs.rviz > $G/rviz_sbs.log 2>&1 & RV=$!
for i in $(seq 30); do xwininfo -name "lab3_sbs.rviz - RViz" > /dev/null 2>&1 && break; sleep 1; done
sleep 4
python3 $L/scripts/move_window.py "lab3_sbs.rviz - RViz" 72 27 600 560
python3 $L/scripts/move_window.py tesse_vnav_scenes 672 27 600 560
sleep 3
# RViz adds "*" to its title once the window is moved, so look windows up by id
rid=$(xwininfo -root -tree | awk '/RViz"/{print $1; exit}')
sid=$(xwininfo -root -tree | awk '/"tesse_vnav_scenes"/{print $1; exit}')
geo() { xwininfo -id "$1" | awk -v k="$2" '$0 ~ k {print $NF}'; }
x1=$(geo $rid "Absolute upper-left X"); y1=$(geo $rid "Absolute upper-left Y"); h1=$(geo $rid "Height:")
x2=$(geo $sid "Absolute upper-left X"); y2=$(geo $sid "Absolute upper-left Y"); w2=$(geo $sid "Width:"); h2=$(geo $sid "Height:")
[ -n "$x1" ] && [ -n "$x2" ] || { echo "window lookup failed"; kill $RV; exit 1; }
y=$(( y1 < y2 ? y1 : y2 )); b1=$((y1+h1)); b2=$((y2+h2)); b=$(( b1 > b2 ? b1 : b2 ))
w=$(( (x2 + w2 - x1) / 2 * 2 )); h=$(( (b - y) / 2 * 2 ))
echo "capture ${w}x${h}+${x1}+${y}  (rviz ${x1},${y1} h${h1}; sim ${x2},${y2} ${w2}x${h2})"
nice -n 10 ffmpeg -y -loglevel error -f x11grab -draw_mouse 0 -framerate 15 -video_size ${w}x${h} -i :0.0+${x1},${y} -t $dur \
  -c:v libx264 -preset ultrafast -crf 23 -pix_fmt yuv420p $V/lab3_rviz_sim_side_by_side.mp4 & FF=$!
sleep 1
roslaunch controller_pkg traj_tracking.launch smooth_start:=true > $G/launch_sbs.log 2>&1 & P=$!
python3 $L/scripts/flight_log.py --mode circle --duration $((dur-2)) --warmup 10 --out $L/results/flight/record_sbs > $V/record_sbs_metrics.txt
wait $FF
ffmpeg -y -loglevel error -f x11grab -draw_mouse 0 -video_size ${w}x${h} -i :0.0+${x1},${y} -frames:v 1 $V/rviz_sim_side_by_side.png
kill -INT $P; wait $P 2>/dev/null
python3 $L/scripts/reset_drone.py > /dev/null
pkill -P $INH 2>/dev/null; kill $RV $INH 2>/dev/null
ls -l $V
