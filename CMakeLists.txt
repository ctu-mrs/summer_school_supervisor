cmake_minimum_required(VERSION 2.8.3)
project(summer_school_supervisor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  nav_msgs
  std_msgs
  mrs_lib
  uvdar_leader_follower
  dynamic_reconfigure
)

###########
## Eigen ##
###########
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include 
  # DEPENDS
  CATKIN_DEPENDS roscpp roslib std_msgs nav_msgs mrs_lib uvdar_leader_follower
  LIBRARIES uvdar_leader_follower
)

set(CMAKE_CXX_FLAGS "-std=c++1z -Wall")

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(summer_school_supervisor src/supervisor.cpp src/message_utils.cpp)
add_dependencies(summer_school_supervisor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(summer_school_supervisor ${catkin_LIBRARIES})
