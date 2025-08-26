cmake_minimum_required(VERSION 3.0.2)
project(car_ctrl)

#include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
#CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
#if(COMPILER_SUPPORTS_CXX11)
#    set(CMAKE_CXX_FLAGS "-std=c++11")
#elseif(COMPILER_SUPPORTS_CXX0X)
#    set(CMAKE_CXX_FLAGS "-std=c++0x")
#else()
#    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a #different C++ compiler.")
#endif()

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  controller_manager
  joint_state_controller
  robot_state_publisher
  hardware_interface
  transmission_interface
  gazebo_ros_control
  gazebo_plugins
  gazebo_ros
  geometry_msgs
  tf
  nav_msgs
)

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
  rospy
  roscpp
  std_msgs
  controller_manager
  joint_state_controller
  robot_state_publisher
  hardware_interface
  transmission_interface
  gazebo_ros_control
  gazebo_plugins
  gazebo_ros
  geometry_msgs
  tf
  nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
add_executable(Car_Plugin src/car_plugin.cpp)

add_executable(world_publisher src/car_world_publisher.cpp)
add_executable(gps_publisher src/car_gps_publisher.cpp)
add_executable(imu_publisher src/car_imu_publisher.cpp)
#add_executable(odom_publisher src/car_odom_publisher.cpp)

add_executable(FR_Steer_Pub src/car_plugin.cpp)
add_executable(FL_Steer_Pub src/car_plugin.cpp)
add_executable(FR_Wheel_Pub src/car_plugin.cpp)
add_executable(FL_Wheel_Pub src/car_plugin.cpp)
add_executable(RR_Wheel_Pub src/car_plugin.cpp)
add_executable(RL_Wheel_Pub src/car_plugin.cpp)

target_link_libraries(Car_Plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

target_link_libraries(world_publisher
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
target_link_libraries(gps_publisher
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
target_link_libraries(imu_publisher
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
#target_link_libraries(odom_publisher
#  ${catkin_LIBRARIES}
#  ${GAZEBO_LIBRARIES}
#)

target_link_libraries(FR_Steer_Pub
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
target_link_libraries(FL_Steer_Pub
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
target_link_libraries(FR_Wheel_Pub
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
target_link_libraries(FL_Wheel_Pub
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
target_link_libraries(RR_Wheel_Pub
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
target_link_libraries(RL_Wheel_Pub
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
