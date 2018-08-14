# DynamicOctomapMapping

This repository contains two nodes allowing to use a UR10 and a kinectv2 camera to 3d mapping of a object being printed in 3d.

How to use this package : 

first connect to the DAB computer using ssh with the command : 

    ssh -X replanner@192.168.1.100
  
then use the following commands to initiate the connection between ros and the UR10 :

    roslaunch ur_modern_driver ur10_bringup.launch robot_ip:=192.168.1.205  roslaunch ur10_moveit_config
    roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch
    roslaunch ur10_moveit_config moveit_rviz.launch config:=true
  
after that you have to launch all nodes required to use the kinect and octomap : 

    roslaunch kinect2_bridge kinect2_bridge.launch depth_method:=opengl reg_nethod:=cpu publish_tf:=true
    roslaunch octomap_server octomap_mapping.launch
  
Finally you can run the two nodes of that repository :

    rosrun dynamicoctomapmapping dynamicoctomapmapping_node
    rosrun dynamicoctomapmapping MoveArm_node
    
Once the mapping is finish you can run the following command to record the map in a file .bt (octomap_binary) or .ot (octomap_full) :
    rosrun octomap_server octomap_saver mapfile.bt
 or
    rosrun octomap_server octomap_saver -f mapfile.ot
