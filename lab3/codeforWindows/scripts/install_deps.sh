#!/bin/bash
# LAB3 系统依赖（要 sudo 密码）。用法：bash scripts/install_deps.sh
set -e
# ROS 官方密钥 + 清华 ROS Noetic 镜像（架构自动识别，x86 电脑是 amd64）
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu focal main" \
  | sudo tee /etc/apt/sources.list.d/ros1.list
sudo apt-get update
sudo apt-get install -y \
  ros-noetic-desktop-full ros-noetic-ackermann-msgs python3-catkin-tools \
  build-essential cmake git wget curl unzip \
  python3-scipy python3-defusedxml python3-pil python3-numpy \
  x11-utils mesa-utils
echo "== 装完了 =="
