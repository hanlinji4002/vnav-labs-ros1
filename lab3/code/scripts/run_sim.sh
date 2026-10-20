#!/bin/bash
# 启动 TESSE 仿真器（x86_64，用 box64 转译）。默认无窗口；加参数 window 试带窗口模式
L=~/Desktop/LAB3
[ -x $L/vnav-builds/lab3/lab3.x86_64 ] || { cd $L/vnav-builds && unzip -oq lab3.zip && chmod +x lab3/lab3.x86_64; }
cd $L/vnav-builds/lab3
export BOX64_LD_LIBRARY_PATH=$L/tools/box64/x64lib
export BOX64_LOG=0
export BOX64_DYNAREC_STRONGMEM=1   # Unity is multithreaded: keep x86 memory ordering (avoids heap corruption)
if [ "$1" = window ]; then
  exec nice -n ${SIM_NICE:-0} $L/tools/box64/build/box64 ./lab3.x86_64 -screen-width ${SIM_W:-600} -screen-height ${SIM_H:-560} -screen-fullscreen 0
else
  exec $L/tools/box64/build/box64 ./lab3.x86_64 -batchmode -nographics
fi
