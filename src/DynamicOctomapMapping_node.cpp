#include <DynamicOctomapMapping_node.h>

using namespace std;
using namespace octomap;
using namespace Eigen;


DynamicOctomapMapping::DynamicOctomapMapping(ros::NodeHandle* nodehandle,  point3d min, point3d max):nh_(*nodehandle){   

    //constructor
    ROS_INFO("in class constructor of DynamicOctomapMapping");

    //initialize Subscribers
    initializeSubscribers();

    //initialize Publishers
    initializePublishers();

    //initialize variables
    BBXMin = min;
    BBXMax = max;
    groundLevel= 0.10;
    lastLayerLevel = 0.0;
    resolution = 0.005;
    sphereRadius = 1;
    lastBestNode = octomath::Vector3(-1.3, -0.25, 0.12);
    lastCameraPose.x = -0.6;
    lastCameraPose.y = -0.6;
    lastCameraPose.z = 0.2;

    computeCBBX();
    printInfo();
    //computeNextCameraOrientation();
    //Some tests

    //generateFakeOctree(0,2,0,2,0,0.1);
    //writeOctomapFile();
    //readOctomapFile();
    //printOctreeInfo();
    //publishOctomap();
    //getOccupiedNodes();
}

void DynamicOctomapMapping::writeOctomapFile(){

    tree_->writeBinary("tree_test.bt");
}


void DynamicOctomapMapping::initializeSubscribers(){

    ROS_INFO("Initializing Subscribers");
    minimal_subscriber_ = nh_.subscribe("/octomap_binary", 1, &DynamicOctomapMapping::subscriberCallback,this); 
}


void DynamicOctomapMapping::initializePublishers(){

    ROS_INFO("Initializing Publishers");
    minimal_publisher_ = nh_.advertise<dynamicoctomapmapping::pose>("new_camera_pose", 1, true); 
}


void DynamicOctomapMapping::subscriberCallback(const octomap_msgs::Octomap msg) {

    AbstractOcTree* tree = octomap_msgs::binaryMsgToMap (msg);
    tree_ = dynamic_cast<OcTree*>(tree);
    find_smallest_occupancy_node();
    publishNextCameraPoseandOrientation();
}


void DynamicOctomapMapping::generateFakeOctree(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max){

    tree_ = new octomap::OcTree(resolution);
    cout << "resolution" << resolution<<  endl;

    for(float z = z_min; z <= z_max; z += resolution)
    {
        for(float y = y_min; y <= y_max; y += resolution)
        {
            for(float x = x_min; x <= x_max; x += resolution)
            {   
                if (z==0){
                    tree_->updateNode(x, y, z, true);
                }
                else {
                    if(x > 0.5 && x < 1.5 && y > 0.5 && y < 1.5){
                        float occ = (float)rand() / (float)RAND_MAX;
                        if(occ > 0.1){
                            tree_->updateNode(x, y, z, false);
                        }
                        else{
                            tree_->updateNode(x, y, z, true);
                        }
                    }
                }
            }
        }
    }
    cout << "fake octree generated" << endl;
}


void DynamicOctomapMapping::publishOctomap(){

    octomap_msgs::Octomap msg;
    OcTree octree = *tree_;
    bool b = octomap_msgs::binaryMapToMsg(octree, msg);
    if(b){
        cout << "je suis la " << endl;
        minimal_publisher_.publish(msg);
    }
}


void DynamicOctomapMapping::readOctomapFile(){

    const string filename = "/home/romain/catkin_ws_kinect/tree_test.bt";
    tree_ = new octomap::OcTree(filename);

    // if this is un file .ot uncomment the 2 lines below and comment the third line
    //octomap::AbstractOcTree* tree = octomap::AbstractOcTree::read(filename);
    //OcTree* tree_ = dynamic_cast<OcTree*>(tree);
}

double DynamicOctomapMapping::difference(octomath::Vector3  leaf1 , octomath::Vector3  leaf2){

    return leaf1.distance(leaf2);
}


void DynamicOctomapMapping::computeCBBX(){

    CBBX = octomath::Vector3((BBXMin.x()+BBXMax.x())/2, (BBXMin.y()+BBXMax.y())/2, BBXMin.z());
}


void DynamicOctomapMapping::find_smallest_occupancy_node(){ 

    double minOccupancy = 1;
    std::cout << "last best node = " << lastBestNode << std::endl;
    point3d newBestNode = lastBestNode;
    double minDistance = difference(BBXMax,BBXMin);
    for(OcTree::leaf_bbx_iterator it = tree_->begin_leafs_bbx(BBXMin,BBXMax), end=tree_->end_leafs_bbx(); it!=end; ++it){
        if (tree_->isNodeOccupied(*it)){     
            double z = it.getZ();
            double occ = it->getOccupancy();
            if (isNewLayer(z) /*&& ((z-lastLayerLevel) <=(2*resolution))*/){
                lastLayerLevel = z;
                //cout << "z = " << z << endl;
                if(occ < minOccupancy){
                    minOccupancy = occ;
                    newBestNode = it.getCoordinate();
                }
                else if(occ == minOccupancy){
                        newBestNode = it.getCoordinate();
                }
            }
        }
    }
    lastBestNode = newBestNode;
    std::cout << "new best node = " << newBestNode << std::endl;
}


geometry_msgs::Point DynamicOctomapMapping::computeNextCameraPose(octomap::point3d p){
    geometry_msgs::Point point;
    double sol, x1, x2, x, y, z;
    double x0 = p.x();
    double y0 = p.y();
    double z0 = p.z();

    double xc = CBBX.x();
    double yc = CBBX.y();
    double zc = CBBX.z();

    double Vx = x0-xc;
    double Vy = y0-yc;
    double Vz = z0-zc;

    double a = Vx*Vx + Vy*Vy + Vz*Vz;
    double b = 2 * ((Vx*Vx) + (Vy*Vy) + (Vz*Vz));
    double c = Vx*Vx + Vy*Vy + Vz*Vz - (sphereRadius*sphereRadius);

    double delta =(b*b) - (4*a*c);

    if(delta == 0){
        sol = -b/(2*a);
    }
    else{
        x1 = (-b -sqrt(delta))/(2*a);
        x2 = (-b +sqrt(delta))/(2*a);
        sol = x1;
    }

    x = Vx*sol + x0;
    y = Vy*sol + y0; 
    z = Vz*sol + z0;

    if( (z <= groundLevel) && (x < xc)){ // if the point is under the ground plane and behind the object
        sol = x2;
        x = Vx*sol + x0;
        y = Vy*sol + y0; 
        z = Vz*sol + z0;
    }
    if( (z < groundLevel) && (x > xc)){ // if the point is under the ground plane and in front of the object
        float diff = zc - z;
        z = zc + diff ;
        //z = -z;    
    }
    if((z >= groundLevel)&& (x < xc)){ // if the point is above the ground plane and behind the object
        float diff = xc - x;
        x = xc + diff;
    }

    point.x = x;
    point.y = y;
    point.z = z;


    //cout << "new camera position = ( " << x << "," << y << "," << z << ")" << endl;

    return point;
}


geometry_msgs::Quaternion DynamicOctomapMapping::computeNextCameraOrientation(geometry_msgs::Point p){

    geometry_msgs::Quaternion orientation;
    //Roll pitch and yaw in Radians
    float pitch, roll, yaw, alpha, beta, gamma;

    double xcam = p.x;
    double ycam = p.y;
    double zcam = p.z;

    double xc = CBBX.x();
    double yc = CBBX.y();
    double zc = CBBX.z();

    roll = 0;

    alpha= acos(   ((-xc*(xcam-xc)) - (yc*(ycam - yc)))/ (sqrt(((xcam - xc)*(xcam - xc))+((ycam - yc)*(ycam - yc))) * sqrt((xc*xc) + (yc*yc)) ));

    if(ycam <= 0){
        yaw = (3.14 - alpha);
    }
    else {
        yaw = -(3.14 - alpha);
    }

    pitch = acos( ( ((xcam-xc)*(xcam-xc)) + ((ycam - yc)*(ycam - yc)) )/  (  sqrt( ((xcam - xc)*(xcam - xc)) + ((ycam - yc)*(ycam - yc)) + ((zcam - zc)*(zcam - zc)) )  * ( sqrt( ((xcam - xc)*(xcam - xc))+((ycam - yc)*(ycam - yc)) ) )));

    //cout << "alpha = "<< alpha <<"\npitch = " << pitch << "\nroll = " << roll << "\nyaw = " << yaw << endl;

//########################## 1st method ##########################//

    // Quaternionf q;
    // q = AngleAxisf(roll, Vector3f::UnitX())
    //     * AngleAxisf(pitch, Vector3f::UnitY())
    //     * AngleAxisf(yaw, Vector3f::UnitZ());

    // orientation.x = q.x();
    // orientation.y = q.y();
    // orientation.z = q.z();
    // orientation.w = q.w();


//########################## 2nd method ##########################//

    // double c1,s1, c2,s2,c3,s3;
    // c1 = cos(pitch/2);
    // c2 = cos(yaw/2);
    // c3 = cos(roll/2);
    // s1 = sin(pitch/2);
    // s2 = sin(yaw/2);
    // s3 = sin(roll/2);

    // orientation.x = s1*s2*c3 + c1*c2*s3;
    // orientation.y = s1*c2*c3 + c1*s2*s3;
    // orientation.z = c1*s2*c3 - s1*c2*s3;
    // orientation.w = c1*c2*c3 - s1*s2*s3;


//########################## 3rd method ##########################//    --> the good one 

    orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch,yaw);

    std::cout << "Orientation = " << std::endl << orientation << std::endl;

    return orientation;
}


void DynamicOctomapMapping::publishNextCameraPoseandOrientation(){
    dynamicoctomapmapping::pose pose;
    geometry_msgs::Point newCameraPose = lastCameraPose;

    pose.goalpose.position = computeNextCameraPose(lastBestNode);
    newCameraPose.x = pose.goalpose.position.x;
    newCameraPose.y = pose.goalpose.position.y;
    newCameraPose.z = pose.goalpose.position.z;

    cout << "lastCameraPose = " << lastCameraPose << "\n newCameraPose = " << newCameraPose << endl;
    octomap::point3d p;
    p.x() = (lastCameraPose.x + newCameraPose.x)/2;
    p.y() = (lastCameraPose.y + newCameraPose.y)/2;
    p.z() = (lastCameraPose.z + newCameraPose.z)/2;
    pose.intermpose.position = computeNextCameraPose(p);

    lastCameraPose = newCameraPose;

    geometry_msgs::Point p2;
    p2.x = p.x();
    p2.y = p.y();
    p2.z = p.z();

    pose.goalpose.orientation = computeNextCameraOrientation(lastCameraPose);
    pose.intermpose.orientation = computeNextCameraOrientation(p2);

    minimal_publisher_.publish(pose);
}

void DynamicOctomapMapping::publishNextCameraPoseandOrientationtest(){
    dynamicoctomapmapping::pose pose;
    geometry_msgs::Point initialCameraPose;
    geometry_msgs::Point finalCameraPose;

    initialCameraPose.x = -0.3;
    initialCameraPose.y = 0.2;
    initialCameraPose.z = 0.3;

    finalCameraPose.x = -0.5;
    finalCameraPose.y = 0.4;
    finalCameraPose.z = 0.3;

    pose.intermpose.position = initialCameraPose;
    pose.intermpose.orientation = computeNextCameraOrientation(initialCameraPose);
    pose.goalpose.position = finalCameraPose;
    pose.goalpose.orientation = computeNextCameraOrientation(finalCameraPose);
    minimal_publisher_.publish(pose);
    sleep(20);
}

bool DynamicOctomapMapping::isNewLayer(double layer){

    if(layer >= (lastLayerLevel)){
        //ROS_INFO("New layer detected, Z = %f", layer );
        return true;
    }
    else{
        //ROS_INFO("No new layer");
        return false;
    }
}


point3d DynamicOctomapMapping::find_closest_node(point3d pos) {

    point3d res = tree_->begin().getCoordinate();
    double dist = 10e9;

    for (OcTree::iterator i = tree_->begin(); i != tree_->end(); i++) {
        double newdist = pos.distance(i.getCoordinate());
        if (newdist < dist) {
            dist = newdist;
            res = i.getCoordinate();
        }
    }
    return res;
}

void DynamicOctomapMapping::updateNodesInBBX(const octomap::point3d& min,
        const octomap::point3d& max, bool occupied) {

    float logodds = tree_->getClampingThresMaxLog() - tree_->getClampingThresMinLog();
    if (!occupied)
        logodds *= -1;

    for(octomap::OcTree::leaf_bbx_iterator it = tree_->begin_leafs_bbx(min,max),
            end=tree_->end_leafs_bbx(); it!= end; ++it){
        tree_->updateNode(it.getKey(), logodds);
    }
}

void DynamicOctomapMapping::printOccupiedNodes(){

    for (OcTree::iterator it = tree_->begin(),
            end = tree_->end(); it != end; ++it){
        if (tree_->isNodeOccupied(*it)){
            cout << "node occupied : " << it.getCoordinate() << endl;
            cout << "node occupancy : " << it->getOccupancy() << endl;
        }
    }
}


const point3d DynamicOctomapMapping::getBBXMin(){

    return BBXMin;
}


const point3d DynamicOctomapMapping::getBBXMax(){

    return BBXMax;
} 


void DynamicOctomapMapping::getBBXMin(const point3d min ){

     BBXMin = min;
}


void DynamicOctomapMapping::getBBXMax(const point3d max){

     BBXMax = max;
} 


void DynamicOctomapMapping::printInfo(){

    cout << "CBBX = " << CBBX << endl;
    cout << "BBXMin = " << BBXMin << endl; 
    cout << "BBXMax = " << BBXMax << endl; 
}


void DynamicOctomapMapping::printOctreeInfo(){

    for(OcTree::leaf_iterator it = tree_->begin_leafs(),end=tree_->end_leafs(); it!= end; ++it){
        cout << "Node center: " << it.getCoordinate() << endl;
        cout << "Node depth: " << it.getDepth() << endl;
        cout << "Node occupancy: " << it->getOccupancy() << endl;
    }
}



/*######################################################################################
################################        MAIN        ####################################
######################################################################################*/

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "DynamicOctomapMapping"); 

    ros::NodeHandle nh; 

    // do not erase
    // point3d BBXMin = point3d(-1.5,-0.25,-0.055);
    // point3d BBXMax = point3d(-0.9,0.25,0.2);

    point3d BBXMin = point3d(-1.6,-0.5,0.10);
    point3d BBXMax = point3d(-1.25,0.5,0.18);


    ROS_INFO("main: instantiating an object of type DynamicOctomapMapping");
    DynamicOctomapMapping DynamicOctomapMapping(&nh,  BBXMin, BBXMax);  
    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
} 
