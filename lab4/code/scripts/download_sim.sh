#!/bin/bash
# 下载 LAB4 仿真器（54.5 MB）。单个连接只有几十 KB/s，所以分 16 段并行下载再拼起来
D=~/Desktop/LAB4/vnav-builds; mkdir -p $D && cd $D || exit 1
U=https://vnav.mit.edu/material/lab4.zip; N=54540713; P=16; S=$(( (N + P - 1) / P ))
for i in $(seq 0 $((P - 1))); do curl -s --retry 5 -r $((i * S))-$(((i + 1) * S - 1)) -o lab4.part$i $U & done; wait
cat $(for i in $(seq 0 $((P - 1))); do echo lab4.part$i; done) > lab4.zip && rm lab4.part*
unzip -tq lab4.zip && unzip -oq lab4.zip && chmod +x lab4/lab4.x86_64 && ls lab4
