#include <record_path_planner/load_path.h>

Load::Load() {
}


Load::~Load() {
}


geometry_msgs::PoseStamped Load::toMsg(double x,double y,double th,ros::Time now){
//    ROS_INFO("test...");
    geometry_msgs::PoseStamped P0;
    P0.header.frame_id = "map";
    P0.header.stamp = now;
    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(th*D2R);
    P0.pose.position.x = x;
    P0.pose.position.y = y;
    P0.pose.position.z = 0;
    P0.pose.orientation = q;
    
    return P0;
}

int Load::init_path(Points waypoint, int num){
    if(num<2){return -1;}
    
    geometry_msgs::PoseStamped from, to;
    ros::Time now = ros::Time::now();
    
    for(int i = 0; i < num; i++){
        int r = 0;
        
        if(i  < num-1){
            from = toMsg(waypoint[i][0],waypoint[i][1],waypoint[i][2],now);
//            cout << "for1:" << i << " " << num << endl;
            to = toMsg(waypoint[i+1][0],waypoint[i+1][1],waypoint[i+1][2],now);
//            cout << "for2:" << i << " " << num << endl;
            r = abs(abs(waypoint[i][2])-abs(waypoint[i+1][2]))<10;
//            cout << "for3:" << i << " " << num << endl;

        }else{
            from = toMsg(waypoint[i][0],waypoint[i][1],waypoint[i][2],now);
            to = toMsg(waypoint[0][0],waypoint[0][1],waypoint[0][2],now);
            r = abs(abs(waypoint[i][2])-abs(waypoint[0][2]))<10;
            
        }
        
        double radius = r==1?-1:radius_;
        Node* path;
        if(i==0){
            path = new Node(i,from,to,radius,NULL,NULL);
        }else{
            path = new Node(i,from,to,radius,list_[i-1],NULL);
        }
        
        list_[i] = path;
   
    }

    list_[0]->setLast(list_[num-1]);
    return 0;
}   

double Load::getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value)
{
    if (value.getType() != XmlRpc::XmlRpcValue::TypeInt && value.getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
        std::string& value_string = value;
        ROS_FATAL("Values in the footprint specification must be numbers. Found value %s.",value_string.c_str());
    }
    return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
}


Points Load::makeWaypointFromXMLRPC(XmlRpc::XmlRpcValue waypoint_xmlrpc){
    int size = waypoint_xmlrpc.size();
    Points waypoint(size);
    for(int i=0;i<size;i++) 
    {
        waypoint[i].resize(3);
    }
    
    for (int i = 0; i < waypoint_xmlrpc.size(); ++i)
    {
        XmlRpc::XmlRpcValue point = waypoint_xmlrpc[i];
        waypoint[i][0] = getNumberFromXMLRPC(point[0]);
        waypoint[i][1] = getNumberFromXMLRPC(point[1]);
        waypoint[i][2] = getNumberFromXMLRPC(point[2]);
    }
     return waypoint;
}

int Load::run(){
    ros::Publisher path_pub_ = nh_.advertise<nav_msgs::Path>("recorded_path", 1, true);
    nh_.param<double>("/load_path/radius", radius_, 2.0);
    Points waypoint;
    XmlRpc::XmlRpcValue waypoint_xmlrpc;
    if (nh_.getParam("/load_path/waypoints", waypoint_xmlrpc))
    {
        waypoint = makeWaypointFromXMLRPC(waypoint_xmlrpc);
    }
    else{
        ROS_FATAL("Get param fatal!");
        return 0;
    }

    int number = waypoint_xmlrpc.size();
    
    nav_msgs::Path path_;
    path_.poses.clear();
    path_.header.frame_id = "map";
    path_.header.stamp = ros::Time::now();
    
    
    if(!init_path(waypoint,number)){
        vector<geometry_msgs::PoseStamped> points_;
        
        for(int i = 0; i < list_.size(); i++){
//            cout << "for:" << i << endl;
            vector<geometry_msgs::PoseStamped>::iterator start,end;
            start = list_[i]->path().begin();
            end = list_[i]->path().end();
            points_.insert(points_.end(),start,end);

        }
        path_.poses = points_;
        
        vector<geometry_msgs::PoseStamped>::iterator it;  
        path_pub_.publish(path_);
    }
    
    ros::spin();
}


int main(int argc, char** argv){
    ros::init(argc, argv, "path_node");
    Load load;
    if(!load.run()){
        return 0;
    };

}
