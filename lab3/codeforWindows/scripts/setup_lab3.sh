#!/bin/bash
# 搭建 ~/LAB3：把 codeforWindows 里的代码放进 catkin 工作区，下载 mav_comm 和仿真器，编译
# 用法（在 clone 下来的仓库里）：bash lab3/codeforWindows/scripts/setup_lab3.sh
set -e
C=$(cd "$(dirname "$0")/.." && pwd)
L=~/LAB3
mkdir -p $L/vnav_ws/src $L/vnav-builds $L/results/logs $L/scripts $L/rviz
for p in controller_pkg tesse-ros-bridge catkin_simple; do
  mkdir -p $L/vnav_ws/src/$p && cp -R $C/$p/. $L/vnav_ws/src/$p/
done
cp $C/scripts/* $L/scripts/ && chmod +x $L/scripts/*
cp $C/rviz/* $L/rviz/
cp $C/env.sh $L/
[ -d $L/vnav_ws/src/mav_comm ] || git clone https://github.com/ethz-asl/mav_comm.git $L/vnav_ws/src/mav_comm
# 仿真器 31.8 MB；国内可能很慢，也可以先用浏览器下好放到 ~/LAB3/vnav-builds/lab3.zip
[ -f $L/vnav-builds/lab3/lab3.x86_64 ] || [ -f $L/vnav-builds/lab3.zip ] || wget -c -O $L/vnav-builds/lab3.zip https://vnav.mit.edu/material/lab3.zip
[ -x $L/vnav-builds/lab3/lab3.x86_64 ] || { cd $L/vnav-builds && unzip -oq lab3.zip && chmod +x lab3/lab3.x86_64; }
source /opt/ros/noetic/setup.bash
cd $L/vnav_ws
catkin init > /dev/null
catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release > /dev/null
catkin build --no-status
echo "== 搭好了：source ~/LAB3/env.sh，然后运行 ~/LAB3/scripts/run_demo.sh =="
