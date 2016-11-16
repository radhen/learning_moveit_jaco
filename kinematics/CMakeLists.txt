cmake_minimum_required(VERSION 2.8.3)
project(kinematics)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  std_msgs
  rviz_visual_tools
  moveit_visual_tools
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(kinematics_model_tutorial
  src/kinematics_model_tutorial.cpp)
target_link_libraries(kinematics_model_tutorial
  ${catkin_LIBRARIES})
