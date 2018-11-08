cmake_minimum_required(VERSION 2.8.3)
project(iri_kinova_demos)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
 COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
  moveit_core
  moveit_ros_planning_interface
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_tutorial
  CATKIN_DEPENDS
  
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(one src/01.cpp)
add_executable(two src/02.cpp)
add_executable(three src/03.cpp)
target_link_libraries(one ${catkin_LIBRARIES})
target_link_libraries(two ${catkin_LIBRARIES})
target_link_libraries(three ${catkin_LIBRARIES})

