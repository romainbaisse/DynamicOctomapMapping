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
    msg_.orientation = tf::createQuaternionMsgFromYaw(-3.14);
    cout << "orientation : " << msg_.orientation << endl;

    move2();

}


void MoveArm::initializeSubscribers(){

    ROS_INFO("Initializing Subscribers");
    minimal_subscriber_ = nh_.subscribe("/new_camera_pose", 1, &MoveArm::subscriberCallback,this);  
}


void MoveArm::subscriberCallback(const geometry_msgs::Pose &msg) {
        /*cout << "position" << endl;
        cout << "x = " << msg.position.x << "y = " << msg.position.y << "z = " << msg.position.z<< endl;
        cout << "orientation" << endl;
        cout << "x = " << msg.orientation.x << "y = " << msg.orientation.y << "z = " << msg.orientation.z << "w = "<< msg.orientation.w << endl;
*/
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
    
    //###################################### add collision object ##################################################

    // kuka bbx 
    moveit_msgs::CollisionObject kuka_bbx;
    kuka_bbx.header.frame_id = move_group.getPlanningFrame();

    kuka_bbx.id = "box1";

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

    kuka_bbx.primitives.push_back(primitive);
    kuka_bbx.primitive_poses.push_back(box_pose);
    kuka_bbx.operation = kuka_bbx.ADD;

    std::vector<moveit_msgs::CollisionObject> kuka_bbxs;
    kuka_bbxs.push_back(kuka_bbx);

    planning_scene_interface.addCollisionObjects(kuka_bbxs);
    //visual_tools.publishText(text_pose, "Add object", WHITE, XLARGE);
    visual_tools.trigger();
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

    // ground bbx 

    moveit_msgs::CollisionObject groundlimit_bbx;
    groundlimit_bbx.header.frame_id = move_group.getPlanningFrame();
    groundlimit_bbx.id = "box2";

    shape_msgs::SolidPrimitive primitive2;
    primitive2.type = primitive2.BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[0] = 2.5;
    primitive2.dimensions[1] = 2.5;
    primitive2.dimensions[2] = 0.01;

    geometry_msgs::Pose ground_pose;
    ground_pose.orientation.w = 1.0;
    ground_pose.position.x = 0;
    ground_pose.position.y = 0;
    ground_pose.position.z = -0.01;

    groundlimit_bbx.primitives.push_back(primitive2);
    groundlimit_bbx.primitive_poses.push_back(ground_pose);
    groundlimit_bbx.operation = groundlimit_bbx.ADD;

    std::vector<moveit_msgs::CollisionObject> groundlimit_bbxs;
    groundlimit_bbxs.push_back(groundlimit_bbx);

    planning_scene_interface.addCollisionObjects(groundlimit_bbxs);
    //visual_tools.publishText(text_pose, "Add object", WHITE, XLARGE);
    visual_tools.trigger();


    //###################################### add collision object ##################################################

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

    cout <<" target_pose = " << target_pose << endl;
    waypoints.push_back(target_pose);  // up and left

    move_group.setMaxVelocityScalingFactor(1);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);


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
