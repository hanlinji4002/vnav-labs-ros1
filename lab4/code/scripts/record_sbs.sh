#!/bin/bash
# 并排录屏（左 RViz，右 Unity 仿真窗口）+ 记录一次竞速
# 用法：record_sbs.sh <名字> [v_max a_max publish_rate control_hz]；增益等用 LAUNCH_ARGS="kr:=16 ..."   要求：虚拟机桌面已登录并解锁
source ~/Desktop/LAB4/env.sh
L=~/Desktop/LAB4; G=$L/results/logs; F=$L/results/flight; V=$L/results/video; mkdir -p $F $V
name=$1; vmax=${2:-8.0}; amax=${3:-4.0}; rate=${4:-5.0}; chz=${5:-1000}
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
# simulator with its window
if ! xwininfo -name tesse_vnav_scenes > /dev/null 2>&1 || ! python3 $L/scripts/sim_ok.py 3; then
  SIM_NICE=${SIM_NICE:-10} $L/scripts/sim_ctl.sh restart window || exit 1
fi
# pointer to the bottom-right corner so no tooltip covers the capture
python3 -c "import ctypes; X=ctypes.cdll.LoadLibrary('libX11.so.6'); X.XOpenDisplay.restype=ctypes.c_void_p; d=ctypes.c_void_p(X.XOpenDisplay(None)); X.XWarpPointer(d,0,ctypes.c_ulong(X.XDefaultRootWindow(d)),0,0,0,0,1275,795); X.XFlush(d)"
python3 $L/scripts/reset_drone.py > /dev/null; sleep 3
python3 $L/scripts/lab4_viz.py > $G/viz_$name.log 2>&1 & VZ=$!
nice -n 10 rviz -d $L/results/rviz/lab4_sbs.rviz > $G/rviz_$name.log 2>&1 & RV=$!
for i in $(seq 30); do xwininfo -name "lab4_sbs.rviz - RViz" > /dev/null 2>&1 && break; sleep 1; done
sleep 4
python3 $L/scripts/move_window.py "lab4_sbs.rviz - RViz" 72 27 600 560
sleep 1
# RViz may refuse to shrink below its minimum width: put the simulator right next to it
rw=$(xwininfo -name "lab4_sbs.rviz - RViz" | awk '/Width:/{print $2}')
python3 $L/scripts/move_window.py tesse_vnav_scenes $((72 + rw)) 27 600 560
roslaunch planner_pkg traj_gen.launch > $G/gen_$name.log 2>&1 & P2=$!   # gates show up in RViz
sleep 4
rid=$(xwininfo -root -tree | awk '/RViz"/{print $1; exit}')
sid=$(xwininfo -root -tree | awk '/"tesse_vnav_scenes"/{print $1; exit}')
geo() { xwininfo -id "$1" | awk -v k="$2" '$0 ~ k {print $NF}'; }
x1=$(geo $rid "Absolute upper-left X"); y1=$(geo $rid "Absolute upper-left Y"); h1=$(geo $rid "Height:")
x2=$(geo $sid "Absolute upper-left X"); y2=$(geo $sid "Absolute upper-left Y"); w2=$(geo $sid "Width:"); h2=$(geo $sid "Height:")
[ -n "$x1" ] && [ -n "$x2" ] || { echo "window lookup failed"; kill $RV $VZ; kill -INT $P2; exit 1; }
y=$(( y1 < y2 ? y1 : y2 )); b1=$((y1+h1)); b2=$((y2+h2)); b=$(( b1 > b2 ? b1 : b2 ))
w=$(( (x2 + w2 - x1) / 2 * 2 )); h=$(( (b - y) / 2 * 2 ))
echo "capture ${w}x${h}+${x1}+${y}"
nice -n 10 ffmpeg -y -loglevel error -f x11grab -draw_mouse 0 -framerate 15 -video_size ${w}x${h} -i :0.0+${x1},${y} \
  -c:v libx264 -preset ultrafast -crf 23 -pix_fmt yuv420p $V/$name.mp4 & FF=$!
sleep 2
top -b -d 3 -n 8 > $G/top_$name.log 2>&1 &
rosbag record -O $F/$name.bag /current_state /desired_state > $G/bag_$name.log 2>&1 & BAG=$!
python3 $L/scripts/race_log.py --out $F/$name --duration 150 > /dev/null & REC=$!
sleep 1
roslaunch ${TRAJ_PKG:-trajectory_generation} traj_following.launch v_max:=$vmax a_max:=$amax publish_rate:=$rate control_hz:=$chz $LAUNCH_ARGS > $G/launch_$name.log 2>&1 & P1=$!
wait $REC
kill -INT $FF; wait $FF 2>/dev/null
ffmpeg -y -loglevel error -sseof -1 -i $V/$name.mp4 -frames:v 1 $V/$name.png
kill -INT $P1 $P2 $BAG; wait $P1 $P2 $BAG 2>/dev/null
python3 $L/scripts/reset_drone.py > /dev/null
kill $RV $VZ 2>/dev/null
cat $F/${name}_summary.txt
ls -l $V/$name.mp4
