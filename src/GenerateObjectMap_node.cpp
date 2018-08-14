#include <GenerateObjectMap_node.h>

using namespace std;
using namespace octomap;


GenerateObjectMap::GenerateObjectMap(ros::NodeHandle* nodehandle):nh_(*nodehandle){   

    //constructor
    ROS_INFO("in class constructor of GenerateObjectMap");

    //initialize Subscribers
    initializeSubscribers();

    //initialize Publishers
    initializePublishers();

    //initialize variables
    
}

void GenerateObjectMap::initializeSubscribers(){

    ROS_INFO("Initializing Subscribers");
    minimal_subscriber_ = nh_.subscribe("/kinect2/hd/points", 1, &GenerateObjectMap::subscriberCallback,this);  
}

void GenerateObjectMap::initializePublishers(){

    ROS_INFO("Initializing Publishers");
    minimal_publisher_ = nh_.advertise<octomap_msgs::Octomap>("object_octomap", 1, true); 
}

void GenerateObjectMap::subscriberCallback(const sensor_msgs::PointCloud2 msg) {

    octomap::Pointcloud pcl;
    const octomap::point3d sensor_origin = octomap::point3d(0,0,0);
    const octomath::Pose6D frame_origin = octomath::Pose6D(0,0,0,0,0,0);

    octomap::point3d mincrop = point3d(-1.5,-0.45,0.10);
    octomap::point3d maxcrop = point3d(-0.9,0.2,0.15);

    octomap::pointCloud2ToOctomap(msg, pcl);
    //crop pcl

    pcl.crop(mincrop, maxcrop);
    tree_->insertPointCloud (pcl, sensor_origin , frame_origin , -1, false, false);
    octomap_msgs::Octomap pub;
    
    bool b = octomap_msgs::binaryMapToMsg(*tree_, pub);
    if(b){
        minimal_publisher_.publish(pub);
    }

}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "GenerateObjectMap"); 

    ros::NodeHandle nh; 

    ROS_INFO("main: instantiating an object of type GenerateObjectMap");
    GenerateObjectMap GenerateObjectMap(&nh); 

    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
} 