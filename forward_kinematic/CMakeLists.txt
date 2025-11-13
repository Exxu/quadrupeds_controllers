cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(qc_forward_kinematic)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ros_gz_sim REQUIRED) # This package now provides the plugin
find_package(robot_state_publisher REQUIRED)
find_package(controller_manager REQUIRED)




ament_package()