cmake_minimum_required(VERSION 2.8.3)
project(pozyx_ros)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    roslib
    rospack
)

add_definitions(-std=c++11)

find_package(cmake_modules REQUIRED)

catkin_package(
   INCLUDE_DIRS
   CATKIN_DEPENDS roscpp rospy roslib rospack
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(KalmanFilter src/KalmanFilter.cpp)
target_link_libraries(KalmanFilter ${catkin_LIBRARIES})

add_executable(pozyx_estimator_node src/PozyxEstimator.cpp)
target_link_libraries(pozyx_estimator_node KalmanFilter ${catkin_LIBRARIES})