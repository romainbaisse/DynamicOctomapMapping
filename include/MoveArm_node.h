
#ifndef MOVEARM_H_
#define MOVEARM_H_

#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> 
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_state/robot_state.h>

#include <rviz_visual_tools/rviz_visual_tools.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <dynamicoctomapmapping/pose.h>

class MoveArm{

    public:
        MoveArm(ros::NodeHandle* nodehandle);

    private:
        ros::NodeHandle nh_; 

        ros::Subscriber minimal_subscriber_; 

		dynamicoctomapmapping::pose msg_;

        void initializeSubscribers(); 

        void subscriberCallback(const dynamicoctomapmapping::pose & msg); 

		void move();
		
		void move2();
	}; 

#endif