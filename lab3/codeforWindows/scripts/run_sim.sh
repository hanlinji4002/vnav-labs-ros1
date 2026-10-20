#!/bin/bash
# 启动 TESSE 仿真器（x86_64 直接运行）。默认无窗口；加参数 window 带 Unity 窗口
# 窗口大小：SIM_W / SIM_H（默认 960x720）；SIM_PREFIX 可在程序前加启动器（x86_64 上留空）
L=~/LAB3
[ -x $L/vnav-builds/lab3/lab3.x86_64 ] || { cd $L/vnav-builds && unzip -oq lab3.zip && chmod +x lab3/lab3.x86_64; }
cd $L/vnav-builds/lab3
if [ "$1" = window ]; then
  exec nice -n ${SIM_NICE:-0} $SIM_PREFIX ./lab3.x86_64 -screen-width ${SIM_W:-960} -screen-height ${SIM_H:-720} -screen-fullscreen 0
else
  exec nice -n ${SIM_NICE:-0} $SIM_PREFIX ./lab3.x86_64 -batchmode -nographics
fi
