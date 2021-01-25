#ifndef LOAD_PATH_H
#define LOAD_PATH_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <record_path_planner/node.h>
//#include <vector>

#define D2R M_PI/180
using namespace std;
typedef vector<vector<double>>  Points;

class Load{
public:
    Load();
    
    ~Load();
    
    int init_path(Points waypoint, int num);
    int run();
    double getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value);
    Points makeWaypointFromXMLRPC(XmlRpc::XmlRpcValue waypoint_xmlrpc);
    geometry_msgs::PoseStamped toMsg(double x,double y,double th,ros::Time now);
private:
    ros::NodeHandle nh_;
    std::map<int, Node*> list_;
    double radius_;
};

#endif //LOAD_PATH_H
