#include <MoveArm_node.h>

using namespace std;
using namespace moveit;
using namespace rviz_visual_tools;

MoveArm::MoveArm(ros::NodeHandle* nodehandle):nh_(*nodehandle){   

    // constructor
    ROS_INFO("in class constructor of MoveArm");

    initializeSubscribers();
    
    // initializePublishers();

    msg_.position.x = -0.5;
    msg_.position.y = 0.45;
    msg_.position.z = 0.3;

    move2();

}


void MoveArm::initializeSubscribers(){

    ROS_INFO("Initializing Subscribers");
    minimal_subscriber_ = nh_.subscribe("/new_camera_pose", 1, &MoveArm::subscriberCallback,this);  
}


void MoveArm::subscriberCallback(const geometry_msgs::Pose &msg) {
        cout << "position" << endl;
        cout << "x = " << msg.position.x << "y = " << msg.position.y << "z = " << msg.position.z<< endl;
        cout << "orientation" << endl;
        cout << "x = " << msg.orientation.x << "y = " << msg.orientation.y << "z = " << msg.orientation.z << "w = "<< msg.orientation.w << endl;

        msg_.position.x = msg.position.x;
        msg_.position.y = msg.position.y;
        msg_.position.z = msg.position.z;
        msg_.orientation.x = msg.orientation.x;
        msg_.orientation.y = msg.orientation.y;
        msg_.orientation.z = msg.orientation.z;
        msg_.orientation.w = msg.orientation.w;


}


void MoveArm::move(){
    //cout << "je suis la" << endl;


    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup("manipulator");

    moveit_visual_tools::MoveItVisualTools visual_tools("world","/rviz_visual_markers");
    visual_tools.deleteAllMarkers();

    visual_tools.loadRemoteControl();

    visual_tools.trigger();

    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    // add collisionh object
/*
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();

    collision_object.id = "box1";


    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.28;
    primitive.dimensions[1] = 1.28;
    primitive.dimensions[2] = 1.28;

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = -1.4;
    box_pose.position.y = 0;
    box_pose.position.z = 0.64;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    planning_scene_interface.addCollisionObjects(collision_objects);
    //visual_tools.publishText(text_pose, "Add object", WHITE, XLARGE);
    visual_tools.trigger();
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

*/


    robot_state::RobotState start_state(*move_group.getCurrentState());
    geometry_msgs::PoseStamped state = move_group.getCurrentPose();

    geometry_msgs::Pose start_pose = state.pose;
    cout <<" start_pose = " << start_pose << endl;

    start_state.setFromIK(joint_model_group, start_pose);
    move_group.setStartState(start_state);

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(start_pose);

    geometry_msgs::Pose target_pose = start_pose;

    target_pose.position.x = msg_.position.x;
    target_pose.position.y = msg_.position.y;
    target_pose.position.z = msg_.position.z;
    target_pose.orientation.x = msg_.orientation.x;
    target_pose.orientation.y = msg_.orientation.y;
    target_pose.orientation.z = msg_.orientation.z;
    target_pose.orientation.w = msg_.orientation.w;

/*
    target_pose.position.x = -0.5;
    target_pose.position.y = 0;
    target_pose.position.z =0.8;*/

    cout <<" target_pose = " << target_pose << endl;
    waypoints.push_back(target_pose);  // up and left

    move_group.setMaxVelocityScalingFactor(0.1);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    /*visual_tools.deleteAllMarkers();
    //visual_tools.publishText(text_pose, "Joint Space Goal", WHITE, XLARGE);
    visual_tools.publishPath(waypoints, LIME_GREEN, SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), SMALL);
    visual_tools.trigger();
*/

    plan.trajectory_ = trajectory;
    /*cout << "if the path looks good enter one key and then enter" << endl;

    string text;
    cin >> text;

    if(!text.empty()){
        move_group.execute(plan);

    }*/
    move_group.execute(plan);

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
