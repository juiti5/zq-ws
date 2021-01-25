#include <control_detection/control.h>
using namespace std;

Control::Control() {
}


Control::~Control() {
}


void Control::cmd_callback(const geometry_msgs::Twist::ConstPtr &msg){
    cmd_msg_ = *msg;
}

    
void Control::path_callback(const nav_msgs::Path::ConstPtr &msg){
    path_msg_ = *msg;
}


void Control::amcl_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
//    ROS_INFO("pose:x:%.2f, y:%.2f", amcl_msg_.pose.pose.position.x,amcl_msg_.pose.pose.position.y);
    amcl_msg_ = *msg;
}

double norm2(const geometry_msgs::PoseStamped &from, geometry_msgs::PoseStamped &to) {
    double delta_x = from.pose.position.x - to.pose.position.x;
    double delta_y = from.pose.position.y - to.pose.position.y;
    return delta_x * delta_x + delta_y * delta_y;
}
        
        
int Control::min_distance_index(const geometry_msgs::PoseStamped &from, nav_msgs::Path &path) {
    int index = -1;
    double min_value = std::numeric_limits<double>::max();
    double dis;
    for (size_t i = 0; i < path.poses.size(); i++) {
        dis = norm2(from, path.poses.at(i));
        if (dis < min_value) {
            min_value = dis;
            index = static_cast<int>(i);
        }
    }
    return index;
}

int Control::cycle_run(){
    if (path_msg_.poses.empty()) {
        ROS_INFO("POSE EMPTY!");
        return -1;
    }
//    ROS_INFO("size:%lu", path_msg_.poses.size());
    int start_index, goal_index, dir;
    geometry_msgs::PoseStamped start;
    start.pose = amcl_msg_.pose.pose;
//    const geometry_msgs::PoseStamped &p = reinterpret_cast<const geometry_msgs::PoseStamped &>(start.pose);
//    cout << amcl_msg_ << endl;
    start_index = min_distance_index(start, path_msg_);
//    ROS_INFO("start_index:%d, x:%.2f, y:%.2f", start_index,start.pose.position.x,start.pose.position.y);
    
    goal_index = start_index;
    for(int i = 0; i < 100; i++){
//        ROS_INFO("i:%d, goal_index:%d", i,goal_index);
        goal_index++;
        if(goal_index >= path_msg_.poses.size()-1){
//            ROS_INFO("<i:%d, goal_index:%d>", i,goal_index);
            goal_index = 0;
        }
    }
//    ROS_INFO("goal_index:%d", goal_index);
    
    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "map";
    goal.pose = path_msg_.poses.at(static_cast<unsigned long>(goal_index)).pose;
    
    
    goal_pub_.publish(goal);
    return 0;
}


void Control::cycle_callback(const ros::TimerEvent &){
    if(cycle_){
        cycle_run();
    }
}


void Control::check_callback(const ros::TimerEvent &){
    double cx,cy;
    tf::StampedTransform local_transform;
    try{
        tf_.lookupTransform(map_frame_, base_frame_,
                               ros::Time(0), local_transform);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    
    cx = local_transform.getOrigin().x();
    cy = local_transform.getOrigin().y();
    
    geometry_msgs::Twist cmd;
    if(cx > 2 && cx < 2.5 && cy > -1 && cy < 1){
        ROS_INFO("1_pose:x:%.2f, y:%.2f", cx,cy);
        if(tl_msg_.light_color == 0){
            ROS_INFO("STOP!");
            cmd.linear.x = 0;
            cmd.angular.z = 0;
        }
        else if(tl_msg_.light_color == 1)
        {
            ROS_INFO("RUN!");
            cmd = cmd_msg_;
        }
    }
    else if(cx > 13 && cx < 16 && cy > 2.5 && cy < 3){
        ROS_INFO("2_pose:x:%.2f, y:%.2f", cx,cy);
        if(tl_msg_.light_color == 0){
            ROS_INFO("STOP!");
            cmd.linear.x = 0;
            cmd.angular.z = 0;
        }
        else if(tl_msg_.light_color == 1)
        {
            ROS_INFO("RUN!");
            cmd = cmd_msg_;
        }
    }
    else if(cx > 6 && cx < 6.5 && cy > 8 && cy < 11){
        ROS_INFO("3_pose:x:%.2f, y:%.2f", cx,cy);
        if(tl_msg_.light_color == 0){
            ROS_INFO("STOP!");
            cmd.linear.x = 0;
            cmd.angular.z = 0;
        }
        else if(tl_msg_.light_color == 1)
        {
            ROS_INFO("RUN!");
            cmd = cmd_msg_;
        }
    }
    else if(cx > -2 && cx < 0 && cy > 6.5 && cy < 7){
        ROS_INFO("4_pose:x:%.2f, y:%.2f", cx,cy);
        if(tl_msg_.light_color == 0){
            ROS_INFO("STOP!");
            cmd.linear.x = 0;
            cmd.angular.z = 0;
        }
        else if(tl_msg_.light_color == 1)
        {
            ROS_INFO("RUN!");
            cmd = cmd_msg_;
        }
    }
    
    else{
            cmd = cmd_msg_;
    }
        
    cmd_pub_.publish(cmd);
}


void Control::signal_callback(const control_detection::Light::ConstPtr &msg){
    tl_msg_ = *msg;
}


void Control::run() {
    map_frame_ = "map";
    base_frame_ = "base_footprint";
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, true);
    path_sub_ = nh_.subscribe<nav_msgs::Path>("/recorded_path", 10, &Control::path_callback, this);
    cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>("/mb_cmd", 10, &Control::cmd_callback, this);
    amcl_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, &Control::amcl_callback, this);
    signal_sub_ = nh_.subscribe<control_detection::Light>("/traffic_light", 10, &Control::signal_callback, this);
    cycle_timer = nh_.createTimer(ros::Duration(5), &Control::cycle_callback, this);
    check_timer = nh_.createTimer(ros::Duration(0.5), &Control::check_callback, this);
    cycle_ = true;
    ros::spin();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "control_node");
    Control Control;
    Control.run();

    return 0;
}
