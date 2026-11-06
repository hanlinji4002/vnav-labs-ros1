#!/bin/bash
# box 两帧：scripts/run_pair.sh <SIFT|SURF|ORB|FAST> ["{nfeatures: 1500, matcher: flann}"] [名字]；约 8 s 后自动结束
source ~/Desktop/LAB5/scripts/common.sh
d=$1; a=$2; tag=${3:-$d}
timeout -s INT 8 roslaunch lab_5 two_frames_tracking.launch descriptor:=$d params:="${a:-{\}}" save_prefix:=$F/box_${tag}_ > $G/box_$tag.log 2>&1
echo "== box $tag [$a]"; grep -E "Descriptor dim|Avg|samples|rror" $G/box_$tag.log | sed 's/\x1b\[[0-9;]*m//g'
