cmake_minimum_required(VERSION 3.0.2)
project(nlopt)

# LAB4 改动：原版构建时 wget 下载 nlopt 源码；这里改用系统库 libnlopt-cxx-dev
find_package(catkin REQUIRED)
find_path(NLOPT_INCLUDE_DIR nlopt.hpp)
find_library(NLOPT_CXX_LIBRARY nlopt_cxx)
if(NOT NLOPT_INCLUDE_DIR OR NOT NLOPT_CXX_LIBRARY)
  message(FATAL_ERROR "nlopt not found: sudo apt install libnlopt-dev libnlopt-cxx-dev")
endif()
catkin_package(LIBRARIES ${NLOPT_CXX_LIBRARY})
