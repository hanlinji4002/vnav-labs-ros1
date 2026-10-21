cmake_minimum_required(VERSION 3.0.2)
project(trajectory_generation_fast)

find_package(catkin REQUIRED COMPONENTS
  catkin_simple
  cmake_modules
  eigen_catkin
  eigen_conversions
  eigen_checks
  glog_catkin
  mav_msgs
  geometry_msgs
  nav_msgs
  trajectory_msgs
  mav_planning_msgs
  mav_trajectory_generation
  mav_visualization
  nlopt
  roslib
  yaml_cpp_catkin
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/trajectory_generation_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
