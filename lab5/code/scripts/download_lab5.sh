#!/bin/bash
# LAB5 下载：在 Mac 终端运行。先下到 ~/Downloads/lab5_data，再传到 VM；可重复运行（断点续传，做完的跳过）
set -e
VM=evan@192.168.64.5
REMOTE=Desktop/LAB5/downloads
D=~/Downloads/lab5_data
mkdir -p "$D"; cd "$D"
ssh "$VM" "mkdir -p $REMOTE"

get() {  # 文件名 URL
  if ssh "$VM" "test -f $REMOTE/$1"; then echo "VM 上已有 $1，跳过"; return; fi
  [ -f "$1.done" ] || { echo "== 下载 $1"; curl -L --fail -C - --retry 5 --retry-delay 3 -o "$1" "$2"; touch "$1.done"; }
  echo "== 传到 VM: $1"
  scp "$1" "$VM:$REMOTE/$1.part"
  ssh "$VM" "mv $REMOTE/$1.part $REMOTE/$1"
}

B=https://www.dropbox.com/scl/fo/pvme4alm9cjckwloll2tu
K='rlkey=xpqg2fiwuyq5tejd7kxrnfe0k&dl=1'
# 1. 源码（约 60 MB）：SIFT/SURF/BRIEF 所在的 xfeatures2d，系统 OpenCV 里没有
get opencv_contrib-4.2.0.tar.gz https://codeload.github.com/opencv/opencv_contrib/tar.gz/refs/tags/4.2.0
get private.hpp https://raw.githubusercontent.com/opencv/opencv/4.2.0/modules/core/include/opencv2/core/private.hpp
# 2. 数据集（0.8 GB + 1.4 GB）
get 30fps_424x240_2018-10-01-18-35-06.bag "$B/AEwhD8oHNro1R9RGtNvwtt4/lab5/30fps_424x240_2018-10-01-18-35-06.bag?$K"
get vnav-lab5-smooth-trajectory.bag "$B/ACpnEeN28dfJ1ypQwZ_ziS8/lab5/vnav-lab5-smooth-trajectory.bag?$K"
echo "全部完成"
