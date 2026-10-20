#!/bin/bash
# LAB3 剩下的两个下载（不用 sudo）。用法：bash ~/Desktop/LAB3/scripts/download_src.sh
set -e
cd ~/Desktop/LAB3
# 仿真器（31.8 MB，-c 接着上次断点下）
wget -c -O vnav-builds/lab3.zip https://vnav.mit.edu/material/lab3.zip
# box64 v0.4.0 源码（用来在 arm64 上跑 x86_64 仿真器）
[ -d tools/box64 ] || git clone --depth 1 --branch v0.4.0 https://github.com/ptitSeb/box64.git tools/box64
ls -l vnav-builds/lab3.zip; echo "== 下完了 =="
