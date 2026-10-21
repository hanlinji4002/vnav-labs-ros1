cmake_minimum_required(VERSION 3.0.2)
project(yaml_cpp_catkin)

# LAB4 改动：原版构建时 git clone yaml-cpp；这里改用系统库 libyaml-cpp-dev（已装）
find_package(catkin REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)
find_library(YAMLCPP_LIBRARY NAMES ${YAMLCPP_LIBRARIES} HINTS ${YAMLCPP_LIBRARY_DIRS})
catkin_package(LIBRARIES ${YAMLCPP_LIBRARY})
