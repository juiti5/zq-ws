#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
//#include <eigen3/Eigen/Core>
#include <Eigen/Dense>
#include <position_optimization/check_odom.h>
#include <vector>
#include <deque>
#include <std_msgs/Float32.h>
#include <iostream>
#include <tf/tf.h> 
#include <visualization_msgs/Marker.h>
#include "sensor_msgs/Imu.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

nav_msgs::Odometry kf_odom_;

MatrixXd pose_kf(MatrixXd z);

std::deque<Eigen::Vector2d> odom_points_;

nav_msgs::Odometry odom_msg_;
std_msgs::Float32 speed_x,speed_y;
sensor_msgs::Imu imu_msg_;

position_optimization::check_odom check_msg_; 

bool first = true;
bool yaw_init = true;

ros::Publisher odom_kf_pub;
ros::Publisher marker_pub;
visualization_msgs::Marker points;

MatrixXd X;  /* state */
MatrixXd F;  /* x(n)=F*x(n-1)+B*u(n),u(n)~N(0,q) */
MatrixXd B;  /*B为内部控制量*/
MatrixXd H;  /* z(n)=H*x(n)+w(n),w(n)~N(0,r)   */
MatrixXd Q;  /* process(predict) noise convariance */
MatrixXd R;  /* measure noise convariance */
MatrixXd P;  /* estimated error convariance */
MatrixXd K;  /*kalman 增益*/
double q = 0.1;
double r = 0.01;
double delta_time;

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  odom_msg_ = *msg.get();
}

void uwb_callback(const position_optimization::check_odom::ConstPtr &msg)
{
  check_msg_ = *msg.get();
}

void speed_callback(const std_msgs::Float32::ConstPtr &msg)
{
  speed_x = *msg.get();
}

void yaw_callback(const std_msgs::Float32::ConstPtr &msg)
{
  speed_y = *msg.get();
}

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
  imu_msg_ = *msg.get();
}
void init()
{
  delta_time=0;
  X=MatrixXd(2, 1);
  F=MatrixXd(2, 2);
  B=MatrixXd(3, 3);
  H=MatrixXd(1, 2);
  Q=MatrixXd(2, 2);
  R=MatrixXd(1,1);
  P=MatrixXd(2, 2);
  K=MatrixXd(2,2);
  
  X<<0,0;

  F<<1,(double)delta_time,
  0,1;

  B.setZero();

  H<<1,0;

  Q<<q,0,
  0,q;
 // cout << "Q:  \n" << Q << endl;
  R<<r;
 // cout << "R:  \n" << R << endl;
  P<<0.1,0,
  0,0.1;

  K<<1,0,
  0,1; 
}

void kf_callback(const ros::TimerEvent &)
{
    MatrixXd zr;
    MatrixXd zf;
    MatrixXd rr;
    MatrixXd rf;
    zr=MatrixXd(1, 1);
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom_msg_.pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    /*
    tf::Quaternion quat1;
tf::quaternionMsgToTF(odom_msg_.pose.pose.orientation, quat1);
double roll1, pitch1, yaw1;
tf::Matrix3x3(quat1).getRPY(roll1, pitch1, yaw1);
*/
/*
    static double init ;
    if(yaw_init)
    {
      init = yaw;
      yaw_init = false;
    }
    */
    zr<<yaw;
    rr = pose_kf(zr);
    //cout << "yaw：\n" << yaw-init << endl;

    double x= odom_msg_.pose.pose.position.x;
    double y = odom_msg_.pose.pose.position.y;
    double kf_yaw = rr(0,0);

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(kf_yaw);
    kf_odom_.pose.pose.orientation = odom_quat;
    kf_odom_.header.frame_id = "wheel_odom"; 
    kf_odom_.child_frame_id = "base_link";
    kf_odom_.header.stamp = ros::Time::now();
    kf_odom_.pose.pose.position.x = x;
    kf_odom_.pose.pose.position.y = y;
    kf_odom_.pose.pose.position.z = 0;
		odom_kf_pub.publish(kf_odom_); 

    geometry_msgs::Point p;
    p.x = rr(0,0);
    p.y = rr(1,0);
    points.header.frame_id = "wheel_odom";
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.05;
    points.scale.y = 0.05;    
    points.color.b = 1.0f;
    points.color.a = 1.0;
    points.points.push_back(p);
    marker_pub.publish(points);
    ROS_INFO("Y:%.2F,YK:%.2F",yaw,kf_yaw);
}
MatrixXd pose_kf(MatrixXd z)
{
  static ros::Time last_time, now;
  now = ros::Time::now();
  delta_time = (now - last_time).toSec();
  if(first)
  {
    delta_time = 0;
    first = false;
  }
  double vth = imu_msg_.angular_velocity.z;
  X(1,0) = vth;
 // F << 1,0,(double)delta_time * cos(speed_y.data), 0,1,(double)delta_time * sin(speed_y.data), 0,0,1;
  F<<1,(double)delta_time,
  0,1;
 
//  B<<0.5*delta_time*delta_time,delta_time;
        /* Predict */
  double u=0;
  //cout << "1F：\n" << F << endl;
  //cout << "1X：\n" << X << endl;
  X = F*X;/*+B*u;*/
 // cout << "X:  \n" << X << endl;

  P = F*P*F.transpose() + Q;
  /*
  cout << "2F:  \n" << F << endl;
  cout << "2Q:  \n" << Q << endl;
  cout << "2P:  \n" << P << endl;
  */
        /*update*/
//  cout << "3HT:  \n" << H*P*H.transpose() << endl;
//  cout << "3T:  \n" << R << endl;

  K = P*H.transpose()*((H*P*H.transpose()+R).inverse());
 
/*
  cout << "3P:  \n" << P << endl;
  cout << "3H:  \n" << H << endl;
  cout << "3R:  \n" << R << endl;
  cout << "3K:  \n" << K << endl;
*/
//cout << "3HT:  \n" << H*P*H.transpose() << endl;
//cout << "3T:  \n" << R << endl;
  X = X+K*(z-H*X);
/*
  cout << "4X:  \n" << X << endl;
  cout << "4K:  \n" << K << endl;
  cout << "4Z:  \n" << z << endl;
*/
  P = P - K*H*P;
  /*
  cout << "5P:  \n" << P << endl;
  cout << "5K:  \n" << K << endl;
  cout << "5H:  \n" << H << endl;
  */
  last_time = now;
  return X;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_kf_node");
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");
    private_node.param<double>("q", q, 0);
    private_node.param<double>("r", r, 0.1);
    odom_kf_pub = node.advertise<nav_msgs::Odometry>("kf_odom", 10);
    marker_pub = node.advertise<visualization_msgs::Marker>("kf_marker", 10);
    
    ros::Subscriber odom_sub = node.subscribe<nav_msgs::Odometry>("/odom", 10, &odom_callback);
    ros::Subscriber imu_subscribe = node.subscribe<sensor_msgs::Imu>("imu", 10, &imu_callback);
    
    ros::Subscriber uwb_sub = node.subscribe< position_optimization::check_odom>("/odom_check", 10, &uwb_callback);
    ros::Subscriber vx_sub = node.subscribe<std_msgs::Float32>("/speed_x", 10, &speed_callback);
    ros::Subscriber vy_sub = node.subscribe<std_msgs::Float32>("/speed_y", 10, &yaw_callback);
    
    ros::Timer kf_timer = node.createTimer(ros::Duration(0.2), &kf_callback);
    init();
    
    ros::spin();
    
    return 0;
}
