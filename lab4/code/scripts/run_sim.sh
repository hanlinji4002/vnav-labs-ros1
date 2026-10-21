#!/bin/bash
# 启动 LAB4 仿真器（x86_64，用 LAB3 里编好的 box64 转译）。默认无窗口；加参数 window 带窗口
B=~/Desktop/LAB3/tools/box64
cd ~/Desktop/LAB4/vnav-builds/lab4
export BOX64_LD_LIBRARY_PATH=$B/x64lib
export BOX64_LOG=0
export BOX64_DYNAREC_STRONGMEM=1   # Unity is multithreaded: keep x86 memory ordering (avoids heap corruption)
if [ "$1" = window ]; then
  exec nice -n ${SIM_NICE:-0} $B/build/box64 ./lab4.x86_64 -screen-width ${SIM_W:-600} -screen-height ${SIM_H:-560} -screen-fullscreen 0
else
  exec nice -n ${SIM_NICE:-0} $B/build/box64 ./lab4.x86_64 -batchmode -nographics
fi
