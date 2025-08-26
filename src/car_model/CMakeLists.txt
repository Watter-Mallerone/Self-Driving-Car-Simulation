cmake_minimum_required(VERSION 3.0.2)
project(car_model)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
  robot_state_publisher
  joint_state_publisher
  joint_state_publisher_gui
  rviz
  tf2
  urdf
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
