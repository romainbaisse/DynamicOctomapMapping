
#ifndef GENERATEOBJECTMAP_H_
#define GENERATEOBJECTMAP_H_

#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/Pointcloud.h>
#include <octomap/math/Pose6D.h>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomapRequest.h>
#include <octomap_msgs/GetOctomapResponse.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/Octomap.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <octomap_ros/conversions.h>


class GenerateObjectMap{

    public:
        GenerateObjectMap(ros::NodeHandle* nodehandle);

    private:
        ros::NodeHandle nh_; 

        ros::Subscriber minimal_subscriber_; 

        ros::Publisher minimal_publisher_;

		octomap::OcTree* tree_;

        void initializeSubscribers(); 

        void initializePublishers(); 

        void subscriberCallback(const sensor_msgs::PointCloud2 msg); 


	}; 

#endif