#include <DynamicOctomapMapping_node.h>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomapRequest.h>
#include <octomap_msgs/GetOctomapResponse.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/math/Vector3.h>
#include <string>


using namespace std;
using namespace octomap;



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
    resolution = 0.05;
    sphereRadius = 1.1;
    lastBestNode = octomath::Vector3(-0.9, -0.25,-0.055);
    lastCameraPose = octomath::Vector3(0.2,0.2,0.2);
    computeCBBX();

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
    minimal_publisher_ = nh_.advertise<geometry_msgs::Point>("my_topic2", 1, true); 
}


void DynamicOctomapMapping::subscriberCallback(const octomap_msgs::Octomap msg) {

    AbstractOcTree* tree = octomap_msgs::binaryMsgToMap (msg);
    tree_ = dynamic_cast<OcTree*>(tree);
    find_smallest_occupancy_node();
    computeNextCameraPose(CBBX, pos);
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
            if (isNewLayer(z)){
                if(occ < minOccupancy){
                    minOccupancy = occ;
                    newBestNode = it.getCoordinate();
                }
                else if(occ == minOccupancy){
                    if(difference(newBestNode,it.getCoordinate()) > minDistance)
                        newBestNode = it.getCoordinate();
                }
            }
        }
    }
    lastBestNode = newBestNode;
    std::cout << "new best node = " << newBestNode << std::endl;
}


void DynamicOctomapMapping::computeNextCameraPose(octomap::point3d centerBBX, double* pos){
    geometry_msgs::Point point;
    double sol, x1, x2, x, y, z;
    double x0 = lastBestNode.x();
    double y0 = lastBestNode.y();
    double z0 = lastBestNode.z();

    double xc = centerBBX.x();
    double yc = centerBBX.y();
    double zc = centerBBX.z();

    double Vx = x0-xc;
    double Vy = y0-yc;
    double Vz = z0-zc;

    double a = Vx*Vx + Vy*Vy + Vz*Vz;
    double b = 2 * ((Vx*Vx) + (Vy*Vy) + (Vz*Vz));
    double c = (x0-xc)*(x0-xc) + (y0-yc)*(y0-yc) + (z0-zc)*(z0-zc) - (sphereRadius*sphereRadius);

    double delta =(b*b) - (4*a*c);
    if(delta <0)
        std::cout << "Solutions complexes" << std::endl;
    else if(delta == 0){
        std::cout << "une seule solution réelle" << std::endl;
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

    minimal_publisher_.publish(point);
    sleep(10);
    cout << "new camera position = ( " << x << "," << y << "," << z << ")" << endl;
}


bool DynamicOctomapMapping::isNewLayer(double layer){

    if(layer >= (lastLayerLevel + resolution)){
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





int main(int argc, char** argv) 
{
    ros::init(argc, argv, "DynamicOctomapMapping"); 

    ros::NodeHandle nh; 

    // do not erase
    // point3d BBXMin = point3d(-1.5,-0.25,-0.055);
    // point3d BBXMax = point3d(-0.9,0.25,0.2);

    point3d BBXMin = point3d(-1.85,-0.4,0.10);
    point3d BBXMax = point3d(-0.9,0.4,0.55);


    ROS_INFO("main: instantiating an object of type DynamicOctomapMapping");
    DynamicOctomapMapping DynamicOctomapMapping(&nh,  BBXMin, BBXMax);  
    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
} 
