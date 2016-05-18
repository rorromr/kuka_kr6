cmake_minimum_required(VERSION 2.8.3)
project(kuka_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  tf
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    geometry_msgs
    sensor_msgs
    urdfdom
    tf
  DEPENDS
    python_orocos_kdl
    urdfdom
)
