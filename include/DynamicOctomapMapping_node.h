
#ifndef DYNAMICOCTOMAPMAPPING_H_
#define DYNAMICOCTOMAPMAPPING_H_

#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <cmath>
#include <iostream>

#include <ros/ros.h> 
#include <std_msgs/Bool.h> 
#include <std_msgs/String.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/math/Vector3.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <dynamicoctomapmapping/pose.h>
class DynamicOctomapMapping{

    public:
        DynamicOctomapMapping(ros::NodeHandle* nodehandle, octomap::point3d min, octomap::point3d max);
        const octomap::point3d getBBXMin();
        const octomap::point3d getBBXMax(); 
        void getBBXMin(const octomap::point3d min);
        void getBBXMax(const octomap::point3d max);

    private:
        ros::NodeHandle nh_; 

        ros::Subscriber minimal_subscriber_; 

        ros::Publisher  minimal_publisher_;
        
        octomap::OcTree* tree_;

        double groundLevel;

        double lastLayerLevel;

        double resolution;

        double sphereRadius;

        octomap::point3d CBBX;

        octomap::point3d BBXMin;

        octomap::point3d BBXMax;

        octomap::point3d lastBestNode;

        geometry_msgs::Point lastCameraPose;


        double *pos;

        // initializes subscribers
        void initializeSubscribers(); 

        // initializes publishers
        void initializePublishers();

        // callback function, called each time a new message is publish on the topic it is subcribed 
        void subscriberCallback(const octomap_msgs::Octomap msg); 

        //read an octomap file .ot or .bt and put the result in the attribute tree_
        void readOctomapFile();

        // publish the octomap corresponding to tree_
        void publishOctomap();

        //print informations about the class, CBBX etc..
        void printInfo();

        //print information of the tree_ object
        void printOctreeInfo();

        //print the coordinates and occupancy of the occupied node
        void printOccupiedNodes();

        // write an octomap file .bt from tree_
        void writeOctomapFile();

        // compute if a layer in parameter is a new layer in the octomap 
        bool isNewLayer(double layer);

        // generate an octree ~randomly
        void generateFakeOctree(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max);

        //compute the coordinate of the Bounding Box Center
        void computeCBBX();

        // compute and return the closest point/node to another passed in parameters
        octomap::point3d find_closest_node(octomap::point3d pos);

        // find the leaf/node with the smallest occupancy in the case of a new layer is detected in the Bounding Box
        // if two nodes are similar in terms of occupancy the distance with the lastbestnode is used to select the best point
        void find_smallest_occupancy_node();

        //compute the distance between two nodes
        double difference(octomath::Vector3  leaf1 , octomath::Vector3  leaf2);

        //
        void updateNodesInBBX(const octomap::point3d& min,const octomap::point3d& max, bool occupied);

        // compute the position of the camera on a sphere centered on the CBBX
        geometry_msgs::Point computeNextCameraPose(octomap::point3d p);

        // compute the orientation of the camera 
        geometry_msgs::Quaternion computeNextCameraOrientation(geometry_msgs::Point p);

        // publish the position + orientation of the camera
        void publishNextCameraPoseandOrientation();

        void publishNextCameraPoseandOrientationtest();

}; 

#endif