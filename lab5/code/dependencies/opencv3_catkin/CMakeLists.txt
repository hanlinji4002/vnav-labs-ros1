cmake_minimum_required(VERSION 3.0.2)
project(opencv3_catkin)

# LAB5 改动：原版下载并编译 OpenCV 3.4.2，和 Noetic cv_bridge 用的 OpenCV 4.2 冲突；
# 这里用系统 OpenCV 4.2，只补编 Ubuntu 包里删掉的 SIFT、SURF、BRIEF（contrib 4.2.0 源码）
find_package(catkin REQUIRED)
find_package(OpenCV 4.2 REQUIRED COMPONENTS core imgproc imgcodecs highgui features2d flann calib3d video)

catkin_package(
  INCLUDE_DIRS xfeatures2d/include ${OpenCV_INCLUDE_DIRS}
  LIBRARIES opencv_xfeatures2d ${OpenCV_LIBS}
)

add_library(opencv_xfeatures2d
  xfeatures2d/src/sift.cpp
  xfeatures2d/src/surf.cpp
  xfeatures2d/src/brief.cpp)
target_include_directories(opencv_xfeatures2d PRIVATE xfeatures2d/include ${OpenCV_INCLUDE_DIRS})
target_compile_definitions(opencv_xfeatures2d PRIVATE OPENCV_ENABLE_NONFREE)
target_link_libraries(opencv_xfeatures2d ${OpenCV_LIBS})
