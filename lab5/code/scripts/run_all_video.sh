#!/bin/bash
# 全部视频统计（约 26 分钟）：播放倍率保证慢的方法也不丢帧
cd ~/Desktop/LAB5/scripts
./run_video.sh 30fps SIFT 0.5
./run_video.sh 30fps SURF 0.5
./run_video.sh 30fps ORB 0.5
./run_video.sh 30fps FAST 0.5
./run_video.sh 30fps LK 0.5
./run_video.sh smooth SIFT 0.25
./run_video.sh smooth SURF 0.25
./run_video.sh smooth ORB 0.5
./run_video.sh smooth FAST 0.5
./run_video.sh smooth LK 0.5
echo ALL_DONE
