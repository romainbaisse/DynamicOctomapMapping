#include <MoveArm_node.h>

using namespace std;
using namespace moveit;


MoveArm::MoveArm(ros::NodeHandle* nodehandle):nh_(*nodehandle){   

    // constructor
    ROS_INFO("in class constructor of MoveArm");

    initializeSubscribers();
    
    // initializePublishers();

    move2();

}


void MoveArm::initializeSubscribers(){
    ROS_INFO("Initializing Subscribers");
    // souscrire à un topic qui lit le nouveau point où la caméra doit se déplacer
    minimal_subscriber_ = nh_.subscribe("/my_topic2", 1, &MoveArm::subscriberCallback,this);  
}


void MoveArm::initializePublishers(){
    ROS_INFO("Initializing Publishers");
    // publishing on topic --> my_topic3
    minimal_publisher_ = nh_.advertise<std_msgs::String>("my_topic3", 1, true); 
}


void MoveArm::subscriberCallback(const geometry_msgs::Point &msg) {
    
        msg_.x = msg.x;
        msg_.y = msg.y;
        msg_.z = msg.z;
}


void MoveArm::move(){
    cout << "je suis la" << endl;
    ros::AsyncSpinner spinner(6);
    spinner.start();
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    move_group.setMaxAccelerationScalingFactor(0.1);
    move_group.setMaxVelocityScalingFactor(0.1);
    geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup("manipulator");

    geometry_msgs::Pose target_pose1;
    //target_pose1.orientation.w = 1.0;
    target_pose1.position.x = msg_.x;
    target_pose1.position.y = msg_.y;
    target_pose1.position.z = msg_.z;

    // target_pose1.position.x = current_pose.pose.position.x - 0.05;
    // target_pose1.position.y = current_pose.pose.position.y;
    // target_pose1.position.z = current_pose.pose.position.z;
    
    target_pose1.orientation = current_pose.pose.orientation;
    move_group.setPoseTarget(target_pose1);
    cout << " current pose =" << current_pose << endl;
    cout << " target pose =" << target_pose1 << endl;

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS); 

    // bool success =(move_group.asyncExecute(my_plan)== moveit::planning_interface::MoveItErrorCode::SUCCESS);

    cout << "if plan looks correct press one random key and then Enter" << endl;
    string enter;
    cin >> enter; 

    if(!enter.empty()){
        //move_group.asyncExecute(my_plan);
        move_group.move();
    }

    // time to visualize the plan on rviz
    sleep(10);

}


void MoveArm::move2(){
    while(ros::ok()){
        move();
    }
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "MoveArm"); 

    ros::NodeHandle nh; 

    ROS_INFO("main: instantiating an object of type MoveArm");
    MoveArm MoveArm(&nh); 

    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
} 
