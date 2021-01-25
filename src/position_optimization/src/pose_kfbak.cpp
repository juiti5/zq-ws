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

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

nav_msgs::Odometry kf_odom_;

MatrixXd pose_kf(MatrixXd x, MatrixXd z, MatrixXd f, MatrixXd p, MatrixXd q, MatrixXd r, MatrixXd k, MatrixXd h);

std::deque<Eigen::Vector2d> odom_pf_;

nav_msgs::Odometry odom_msg_;
std_msgs::Float32 speed_x,speed_y;

position_optimization::check_odom check_msg_; 

bool first = true;

ros::Publisher odom_kf_pub;
ros::Publisher point_front_pub;
ros::Publisher point_rear_pub;
visualization_msgs::Marker pf;
visualization_msgs::Marker pr;

MatrixXd X;  /* state */
MatrixXd F;  /* x(n)=F*x(n-1)+B*u(n),u(n)~N(0,q) */
MatrixXd B;  /*B为内部控制量*/
MatrixXd H;  /* z(n)=H*x(n)+w(n),w(n)~N(0,r)   */
MatrixXd Q;  /* process(predict) noise convariance */
MatrixXd R;  /* measure noise convariance */
MatrixXd P;  /* estimated error convariance */
MatrixXd K;  /*kalman 增益*/

MatrixXd YX;  /* state */
MatrixXd YF;  /* x(n)=F*x(n-1)+B*u(n),u(n)~N(0,q) */
MatrixXd YB;  /*B为内部控制量*/
MatrixXd YH;  /* z(n)=H*x(n)+w(n),w(n)~N(0,r)   */
MatrixXd YQ;  /* process(predict) noise convariance */
MatrixXd YR;  /* measure noise convariance */
MatrixXd YP;  /* estimated error convariance */
MatrixXd YK;  /*kalman 增益*/
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
    /*
    MatrixXd z;
    MatrixXd r;
    z=MatrixXd(2, 1);
    z<<check_msg_.x,check_msg_.y;
    r = pose_kf(z);
    //cout << "Z：\n" << z << endl;
    kf_odom_.header.frame_id = "wheel_odom"; 
    kf_odom_.child_frame_id = "base_link";
    kf_odom_.header.stamp = ros::Time::now();
    kf_odom_.pose.pose.position.x = r(0,0);
    kf_odom_.pose.pose.position.y = r(1,0);
    kf_odom_.pose.pose.position.z = 0;
		odom_kf_pub.publish(kf_odom_); 

    ROS_INFO("X:%.2F,Y:%.2F,V:%.2F",r(0,0),r(1,0),r(2,0));
    */
}

void speed_callback(const std_msgs::Float32::ConstPtr &msg)
{
  speed_x = *msg.get();
  float v = speed_x.data;
//  odom_points_.push_back(v);
}

void yaw_callback(const std_msgs::Float32::ConstPtr &msg)
{
  speed_y = *msg.get();
}

void position_init()
{
  delta_time=0;
  X=MatrixXd(4, 1);
  F=MatrixXd(4, 4);
  B=MatrixXd(3, 3);
  H=MatrixXd(2, 4);
  Q=MatrixXd(4, 4);
  R=MatrixXd(2,2);
  P=MatrixXd(4, 4);
  K=MatrixXd(4,4);
  
  X<<0,0,0,0;

  F<<1,0,(double)delta_time, 0,
  0,1,0,(double)delta_time,
  0,0,1,0,
  0,0,0,1;

  B.setZero();

  H<<1,0,0,0,
  0,1,0,0;

  Q<<q,0,0,0,
  0,q,0,0,
  0,0,q,0,
  0,0,0,q;
  cout << "Q:  \n" << Q << endl;
  R<<r,0,0,r;
  cout << "R:  \n" << R << endl;
  P<<0.1,0,0,0,
  0,0.1,0,0,
  0,0,0.1,0,
  0,0,0,0.1;

  K<<1,0,0,0,
  0,1,0,0,
  0,0,1,0,
  0,0,0,1; 
}

void yaw_init()
{
  delta_time=0;
  X=MatrixXd(2, 1);
  F=MatrixXd(2, 2);
  B=MatrixXd(3, 3);
  H=MatrixXd(2, 1);
  Q=MatrixXd(2, 2);
  R=MatrixXd(1,1);
  P=MatrixXd(2, 2);
  K=MatrixXd(2,2);
  
  X<<0,0;

  F<<1,0,(double)delta_time,
  0,1;

  B.setZero();

  H<<1,0,
  0,1;

  Q<<q,0,
  0,q;
  cout << "Q:  \n" << Q << endl;
  R<<r;
  cout << "R:  \n" << R << endl;
  P<<0.1,0,
  0,0.1;

  K<<1,0,
  0,1; 
}

double cov(MatrixXd X, MatrixXd Y)
{
  double px,py,sx,sy,cov;
  for(int i = 0; i < 10; i++)
  {
    px += X(i,0);
    py += Y(i,0);
  }
  px = px/10;
  py = py/10;
    
  for(int i = 0; i < 10; i++)
  {
    sx += (X(i,0) - px) * (Y(i,0) - py);
  }
  cov =  sx/(10-1);
  
  return cov;
}
void kf_callback(const ros::TimerEvent &)
{
    position_init();
    MatrixXd zr;
    MatrixXd zf;
    MatrixXd rr;
    MatrixXd rf;

    double vx = speed_x.data;
    double vy = speed_y.data;
    X(2,0) = vx;
    X(3,0) = vy;

    zr=MatrixXd(2, 1);
    zf=MatrixXd(2, 1);
    zr<<check_msg_.x,check_msg_.y;
    zf<<check_msg_.xx,check_msg_.yy;

    rr = pose_kf(X,zr,F,P,Q,R,K,H);
    rf = pose_kf(X,zf,F,P,Q,R,K,H);
    //cout << "Z：\n" << z << endl;

    double dx = rf(0,0) - rr(0,0);
    double dy = rf(1,0) - rr(1,0);
    double yaw = atan2(dy,dx);

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);
    kf_odom_.pose.pose.orientation = odom_quat;
    kf_odom_.header.frame_id = "wheel_odom"; 
    kf_odom_.child_frame_id = "base_link";
    kf_odom_.header.stamp = ros::Time::now();
    kf_odom_.pose.pose.position.x = rr(0,0);
    kf_odom_.pose.pose.position.y = rr(1,0);
    kf_odom_.pose.pose.position.z = 0;
		odom_kf_pub.publish(kf_odom_); 

    geometry_msgs::Point f;
    f.x = rf(0,0);
    f.y = rf(1,0);
    geometry_msgs::Point r;
    r.x = rr(0,0);
    r.y = rr(1,0);

    pf.header.frame_id = "wheel_odom";
    pf.header.stamp = ros::Time::now();
    pf.ns = "pf_and_lines";
    pf.action = visualization_msgs::Marker::ADD;
    pf.pose.orientation.w = 1.0;
    pf.id = 0;
    pf.type = visualization_msgs::Marker::POINTS;
    pf.scale.x = 0.05;
    pf.scale.y = 0.05;    
    pf.color.b = 1.0f;
    pf.color.a = 1.0;
    pf.points.push_back(f);

    pr.header.frame_id = "wheel_odom";
    pr.header.stamp = ros::Time::now();
    pr.ns = "pr_and_lines";
    pr.action = visualization_msgs::Marker::ADD;
    pr.pose.orientation.w = 1.0;
    pr.id = 1;
    pr.type = visualization_msgs::Marker::POINTS;
    pr.scale.x = 0.05;
    pr.scale.y = 0.05;    
    pr.color.g = 1.0f;
    pr.color.a = 1.0;
    pr.points.push_back(r);

    point_front_pub.publish(pf);
    point_rear_pub.publish(pr);
//    ROS_INFO("X:%.2F,Y:%.2F,V:%.2F",r(0,0),r(1,0),r(2,0));
}
MatrixXd pose_kf(MatrixXd x, MatrixXd z, MatrixXd f, MatrixXd p, MatrixXd q, MatrixXd r, MatrixXd k, MatrixXd h)
{
  static ros::Time last_time, now;
  now = ros::Time::now();
  delta_time = (now - last_time).toSec();
  if(first)
  {
    delta_time = 0;
    first = false;
  }
  // F << 1,0,(double)delta_time * cos(speed_y.data), 0,1,(double)delta_time * sin(speed_y.data), 0,0,1;


//  B<<0.5*delta_time*delta_time,delta_time;
        /* Predict */
  //double u=0;
  //cout << "1F：\n" << F << endl;
  //cout << "1X：\n" << X << endl;
  x = f*x;/*+B*u;*/
 // cout << "X:  \n" << X << endl;

  p = f*p*f.transpose() + q;
  /*
  cout << "2F:  \n" << F << endl;
  cout << "2Q:  \n" << Q << endl;
  cout << "2P:  \n" << P << endl;
  */
        /*update*/
//  cout << "3HT:  \n" << H*P*H.transpose() << endl;
//  cout << "3T:  \n" << R << endl;
  k = p*h.transpose()*((h*p*h.transpose()+r).inverse());
 
/*
  cout << "3P:  \n" << P << endl;
  cout << "3H:  \n" << H << endl;
  cout << "3R:  \n" << R << endl;
  cout << "3K:  \n" << K << endl;
*/
//cout << "3HT:  \n" << H*P*H.transpose() << endl;
//cout << "3T:  \n" << R << endl;
  x = x+k*(z-h*x);
/*
  cout << "4X:  \n" << X << endl;
  cout << "4K:  \n" << K << endl;
  cout << "4Z:  \n" << z << endl;
*/
  p = p - k*h*p;
  /*
  cout << "5P:  \n" << P << endl;
  cout << "5K:  \n" << K << endl;
  cout << "5H:  \n" << H << endl;
  */
  last_time = now;
  return x;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_kf_node");
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");
    private_node.param<double>("q", q, 0);
    private_node.param<double>("r", r, 0.1);
    odom_kf_pub = node.advertise<nav_msgs::Odometry>("kf_odom", 10);
    point_front_pub = node.advertise<visualization_msgs::Marker>("pf_marker", 10);
    point_rear_pub = node.advertise<visualization_msgs::Marker>("pr_marker", 10);
    ros::Subscriber odom_sub = node.subscribe<nav_msgs::Odometry>("/odom", 10, &odom_callback);
    ros::Subscriber uwb_sub = node.subscribe< position_optimization::check_odom>("/odom_check", 10, &uwb_callback);
    ros::Subscriber vx_sub = node.subscribe<std_msgs::Float32>("/speed_x", 10, &speed_callback);
    ros::Subscriber vy_sub = node.subscribe<std_msgs::Float32>("/speed_y", 10, &yaw_callback);
    
    ros::Timer kf_timer = node.createTimer(ros::Duration(0.2), &kf_callback);
    //init();
    
    ros::spin();
    
    return 0;
}
