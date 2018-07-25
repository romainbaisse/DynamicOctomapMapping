cmake_minimum_required(VERSION 2.8.3)
project(my_pkg)
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg octomap octomap_ros octomap_msgs geometry_msgs moveit_ros_control_interface moveit_ros_move_group moveit_ros_planning moveit_ros_planning_interface sensor_msgs
)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${octomap_INCLUDE_DIRS})
include_directories(include ${octomap_msgs_INCLUDE_DIRS})
include_directories(include ${moveit_coreINCLUDE_DIRS})
include_directories(include ${moveit_ros_move_group_coreINCLUDE_DIRS})

add_executable(DynamicOctomapMapping_node src/DynamicOctomapMapping_node.cpp)
target_link_libraries(DynamicOctomapMapping_node ${catkin_LIBRARIES})
add_dependencies(DynamicOctomapMapping_node my_pkg_generate_messages_cpp)

add_executable(MoveArm_node src/MoveArm_node.cpp)
target_link_libraries(MoveArm_node ${catkin_LIBRARIES})
add_dependencies(MoveArm_node my_pkg_generate_messages_cpp)
