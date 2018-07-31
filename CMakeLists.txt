cmake_minimum_required(VERSION 2.8.3)
project(dynamicoctomapmapping)
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg octomap octomap_ros octomap_msgs geometry_msgs moveit_ros_control_interface moveit_ros_move_group moveit_ros_planning moveit_ros_planning_interface sensor_msgs moveit_visual_tools rviz_visual_tools tf
)

## Declare ROS messages and services

## Generate added messages and services

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${octomap_INCLUDE_DIRS})
include_directories(include ${octomap_msgs_INCLUDE_DIRS})
include_directories(include ${moveit_coreINCLUDE_DIRS})
include_directories(include ${moveit_ros_move_group_coreINCLUDE_DIRS})
#include_directories(include ${moveit_visual_toolsINCLUDE_DIRS})

add_executable(dynamicoctomapmapping_node src/DynamicOctomapMapping_node.cpp)
target_link_libraries(dynamicoctomapmapping_node ${catkin_LIBRARIES})
add_dependencies(dynamicoctomapmapping_node my_pkg_generate_messages_cpp)

add_executable(MoveArm_node src/MoveArm_node.cpp)
target_link_libraries(MoveArm_node ${catkin_LIBRARIES})
add_dependencies(MoveArm_node my_pkg_generate_messages_cpp)
