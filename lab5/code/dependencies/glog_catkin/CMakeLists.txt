cmake_minimum_required(VERSION 3.0.2)
project(glog_catkin)

# LAB4 改动：原版构建时从 GitHub 下载 glog 源码；这里改用系统库 libgoogle-glog-dev
find_package(catkin REQUIRED)
find_path(GLOG_INCLUDE_DIR glog/logging.h)
find_library(GLOG_LIBRARY glog)
if(NOT GLOG_INCLUDE_DIR OR NOT GLOG_LIBRARY)
  message(FATAL_ERROR "glog not found: sudo apt install libgoogle-glog-dev")
endif()
catkin_package(LIBRARIES ${GLOG_LIBRARY})
