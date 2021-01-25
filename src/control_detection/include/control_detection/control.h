#ifndef CONTROL_H
#define CONTROL_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <control_detection/Light.h>

class Control {
public:
    Control();

    ~Control();
    
    void path_callback(const nav_msgs::Path::ConstPtr &msg);
    void cmd_callback(const geometry_msgs::Twist::ConstPtr &msg);
    void amcl_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void signal_callback(const control_detection::Light::ConstPtr &msg);
    void cycle_callback(const ros::TimerEvent &);
    int min_distance_index(const geometry_msgs::PoseStamped &from, nav_msgs::Path &path);
    void check_callback(const ros::TimerEvent &);
    int cycle_run();
    
    void run();

private:
    ros::NodeHandle nh_;
    bool cycle_;
    nav_msgs::Path path_msg_;
    geometry_msgs::Twist cmd_msg_;
    geometry_msgs::PoseWithCovarianceStamped amcl_msg_;
    control_detection::Light tl_msg_;
    ros::Timer cycle_timer,check_timer;
    ros::Subscriber path_sub_;
    ros::Subscriber cmd_sub_;
    ros::Subscriber amcl_sub_;
    ros::Subscriber signal_sub_;
    ros::Publisher cmd_pub_;
    ros::Publisher goal_pub_;
    
    std::string map_frame_, base_frame_;
    tf::TransformListener tf_;
};


#endif //CONTROL_H
