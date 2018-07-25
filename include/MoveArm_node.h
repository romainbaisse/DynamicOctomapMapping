
#ifndef EXAMPLE_ROS_CLASS_H_
#define EXAMPLE_ROS_CLASS_H_

#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> 
#include <std_msgs/Bool.h> 
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

#include <std_msgs/String.h>
#include <vector>
#include <string>
#include <tf/transform_listener.h>

#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_state/robot_state.h>

class MoveArm{

    public:
        MoveArm(ros::NodeHandle* nodehandle);

    private:
        ros::NodeHandle nh_; 

        ros::Subscriber minimal_subscriber_; 

        ros::Publisher  minimal_publisher_;

        geometry_msgs::Pose start_pose;

		geometry_msgs::Pose next_pose;

		geometry_msgs::Point msg_;

        void initializeSubscribers(); 

        void initializePublishers();

        void subscriberCallback(const geometry_msgs::Point & msg); 

		void move();
		
		void move2();
	}; 

#endif