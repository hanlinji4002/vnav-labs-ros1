#!/bin/bash
# 视频：scripts/run_video.sh <30fps|smooth> <SIFT|SURF|ORB|FAST|LK> <播放倍率> ["{nfeatures: 1500, matcher: flann}"] [名字]
source ~/Desktop/LAB5/scripts/common.sh
case $1 in
  30fps) bag=$L/downloads/30fps_424x240_2018-10-01-18-35-06.bag ;;
  smooth) bag=$L/downloads/vnav-lab5-smooth-trajectory.bag ;;
  *) echo "unknown bag $1"; exit 1 ;;
esac
d=$2; rate=$3; a=$4; tag=${5:-$d}
roslaunch lab_5 video_tracking.launch path_to_dataset:=$bag descriptor:=$d rate:=$rate params:="${a:-{\}}" save_prefix:=$F/$1_${tag}_ > $G/$1_$tag.log 2>&1
echo "== $1 $tag rate=$rate [$a]"; grep -E "Avg|samples|rror" $G/$1_$tag.log | sed 's/\x1b\[[0-9;]*m//g'
