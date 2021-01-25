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
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

#define PI 3.14159
std::string odom_frame_, base_frame_, map_frame_;
bool pm2o_tf_;
nav_msgs::Odometry kf_odom_;
nav_msgs::Odometry pose_msg_;
MatrixXd pose_kf(MatrixXd xx,MatrixXd z);

std::deque<Eigen::Vector2d> odom_points_;

nav_msgs::Odometry odom_msg_;
std_msgs::Float32 speed_x, speed_y;

position_optimization::check_odom check_msg_;

geometry_msgs::PoseWithCovarianceStamped amcl_msg_;
geometry_msgs::PoseWithCovarianceStamped u2a_msg_;
bool first = true;
ros::Publisher pose_pub_;
ros::Publisher odom_kf_pub;
ros::Publisher marker_pub;
visualization_msgs::Marker points;
tf::Transform transform_;

MatrixXd X;  /* state */
MatrixXd F;  /* x(n)=F*x(n-1)+B*u(n),u(n)~N(0,q) */
MatrixXd JF; /* x(n)=F*x(n-1)+B*u(n),u(n)~N(0,q) */
MatrixXd B;  /*B为内部控制量*/
MatrixXd U;  /*B为内部控制量*/
MatrixXd H;  /* z(n)=H*x(n)+w(n),w(n)~N(0,r)   */
MatrixXd Q;  /* process(predict) noise convariance */
MatrixXd R;  /* measure noise convariance */
MatrixXd P;  /* estimated error convariance */
MatrixXd K;  /*kalman 增益*/

MatrixXd AP; /* measure noise convariance */
MatrixXd UP; /* estimated error convariance */
double q = 0.1;
double r = 0.01;
double wpa, wpu, wya, wyu;
double dt;

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
  //  float v = speed_x.data;
  //  odom_points_.push_back(v);
}

void yaw_callback(const std_msgs::Float32::ConstPtr &msg)
{
  speed_y = *msg.get();
}

void amcl_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  amcl_msg_ = *msg.get();
  double _x = amcl_msg_.pose.pose.position.x;
  double _y = amcl_msg_.pose.pose.position.y;
  /*
  double _qx = amcl_msg_.pose.pose.orientation.x;
  double _qy = amcl_msg_.pose.pose.orientation.y;
  double _qz = amcl_msg_.pose.pose.orientation.z;
  double _qw = amcl_msg_.pose.pose.orientation.w;

  transform_.setOrigin(tf::Vector3(_x, _y, 0.0));
  transform_.setRotation(tf::Quaternion(_qx, _qy, _qz, _qw));
  tf::TransformBroadcaster br_;
  br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "map", "odom"));
  */
  tf::Quaternion quat;
  tf::quaternionMsgToTF(amcl_msg_.pose.pose.orientation, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  AP << _x, _y, yaw;
}

void u2a_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  u2a_msg_ = *msg.get();
  double _x = u2a_msg_.pose.pose.position.x;
  double _y = u2a_msg_.pose.pose.position.y;
  /*
  double _qx = u2a_msg_.pose.pose.orientation.x;
  double _qy = u2a_msg_.pose.pose.orientation.y;
  double _qz = u2a_msg_.pose.pose.orientation.z;
  double _qw = u2a_msg_.pose.pose.orientation.w;

  transform_.setOrigin(tf::Vector3(_x, _y, 0.0));
  transform_.setRotation(tf::Quaternion(_qx, _qy, _qz, _qw));
  tf::TransformBroadcaster br_;
  br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "map", "odom"));
  */
  tf::Quaternion quat;
  tf::quaternionMsgToTF(u2a_msg_.pose.pose.orientation, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  UP << _x, _y, yaw;
}

void init()
{
  dt = 0;
  X = MatrixXd(4, 1);
  F = MatrixXd(4, 4);
  JF = MatrixXd(4, 4);
  B = MatrixXd(4, 2);
  U = MatrixXd(2, 1);
  H = MatrixXd(2, 4);
  Q = MatrixXd(4, 4);
  R = MatrixXd(2, 2);
  P = MatrixXd(4, 4);
  K = MatrixXd(4, 4);
  AP = MatrixXd(3, 1);
  UP = MatrixXd(3, 1);
  //  X << 0, 0, 0, 0;
  X.setZero();
  F << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 0;

  JF << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1;

  B.setZero();
  U.setZero();

  H << 1, 0, 0, 0,
      0, 1, 0, 0;

  Q << q, 0, 0, 0,
      0, q, 0, 0,
      0, 0, q, 0,
      0, 0, 0, q;
  //  cout << "Q:  \n" << Q << endl;
  R << r, 0, 0, r;
  //  cout << "R:  \n" << R << endl;
  P << 0.1, 0, 0, 0,
      0, 0.1, 0, 0,
      0, 0, 0.1, 0,
      0, 0, 0, 0.1;

  K << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1;
}

double cov(MatrixXd X, MatrixXd Y)
{
  double px, py, sx, sy, cov;
  for (int i = 0; i < 10; i++)
  {
    px += X(i, 0);
    py += Y(i, 0);
  }
  px = px / 10;
  py = py / 10;

  for (int i = 0; i < 10; i++)
  {
    sx += (X(i, 0) - px) * (Y(i, 0) - py);
  }
  cov = sx / (10 - 1);

  return cov;
}

void kf_callback(const ros::TimerEvent &)
{
  MatrixXd zr;
  MatrixXd zf;
  MatrixXd rr;
  MatrixXd rf;
  MatrixXd xx;
  zr = MatrixXd(2, 1);
  xx = MatrixXd(2, 1);
  //zf=MatrixXd(2, 1);
  double ux = check_msg_.x;
  double uy = check_msg_.y;
  // ux = abs(ux)>100?100:ux;
  // uy = abs(uy)>100?100:uy;
  double vx = odom_msg_.twist.twist.linear.z;
  double vth = odom_msg_.twist.twist.angular.z;
  xx << vx, vth;
  zr << ux, uy;
  //zf<<check_msg_.xx,check_msg_.yy;
  rr = pose_kf(xx, zr);
  //rf = pose_kf(xx, zf);
  //  cout << "Z：\n" << rr << endl;

  tf::Quaternion quat;
  tf::quaternionMsgToTF(odom_msg_.pose.pose.orientation, quat);
  double o_roll, o_pitch, o_yaw;
  tf::Matrix3x3(quat).getRPY(o_roll, o_pitch, o_yaw);

  static double curr_x, curr_y, last_x, last_y, diff_x, diff_y;
  static double curr_yaw, last_yaw, diff_yaw, real_yaw;
  curr_x = rr(0, 0);
  curr_y = rr(1, 0);
  diff_x = curr_x - last_x;
  diff_y = curr_y - last_y;
  last_x = curr_x;
  last_y = curr_y;

  double dx = check_msg_.xx - rr(0, 0);
  double dy = check_msg_.yy - rr(1, 0);
  double kf_yaw = atan2(diff_y, diff_x);
  //  ROS_INFO("BIG=CY:%.2F",kf_yaw);

  curr_yaw = kf_yaw;
  diff_yaw = abs(curr_yaw - last_yaw);
  diff_yaw = diff_yaw > PI ? 2 * PI - diff_yaw : diff_yaw;

  double total_weight = 30.0, odom_weight = 25.0, uwb_weight = 5.0;
  if (diff_yaw > 0.2)
  {
    //o_yaw = o_yaw<0?o_yaw+2*PI:o_yaw;
    //last_yaw = last_yaw<0?last_yaw+2*PI:last_yaw;
    if (o_yaw > 0 && last_yaw < 0)
    {
      double dy = o_yaw + (-last_yaw);
      double w;
      if (dy > PI)
      {
        w = (2 * PI - dy) / total_weight;
      }
      else if (dy < PI)
      {
        w = dy / total_weight;
      }
      real_yaw = o_yaw + uwb_weight * w;
      real_yaw = real_yaw > PI ? 2 * PI - real_yaw : real_yaw;
    }
    else if (o_yaw < 0 && last_yaw > 0)
    {
      double dy = last_yaw + (-o_yaw);
      double w;
      if (dy > PI)
      {
        w = -(2 * PI - dy) / total_weight;
      }
      else if (dy < PI)
      {
        w = -dy / total_weight;
      }
      real_yaw = o_yaw + uwb_weight * w;
      real_yaw = real_yaw > PI ? 2 * PI - real_yaw : real_yaw;
    }
    else
    {
      real_yaw = (o_yaw * odom_weight + last_yaw * uwb_weight) / total_weight;
    }

    //real_yaw = (o_yaw*15 + last_yaw*5)/20;//(o_yaw*10 + last_yaw*9 + curr_yaw*1)/30;
    //real_yaw = real_yaw > PI ? real_yaw - 2 * PI : real_yaw;
    //ROS_INFO("BIG=CY:%.2F,LY:%.2F,DY:%.2F",curr_yaw,last_yaw,diff_yaw);
    curr_yaw = real_yaw;
  }
  else
  {
    real_yaw = curr_yaw;
    //    ROS_INFO("OK=CY:%.2F,LY:%.2F,DY:%.2F",curr_yaw,last_yaw,diff_yaw);
  }
  last_yaw = curr_yaw;
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(real_yaw);
  kf_odom_.pose.pose.orientation = odom_quat;
  kf_odom_.header.frame_id = odom_frame_;
  kf_odom_.child_frame_id = base_frame_;
  kf_odom_.header.stamp = ros::Time::now();
  kf_odom_.pose.pose.position.x = rr(0, 0);
  kf_odom_.pose.pose.position.y = rr(1, 0);
  kf_odom_.pose.pose.position.z = 0;
  /*
    if (publish_tf_) 
    {
      //std::string car;
      static tf::TransformBroadcaster br_;
     // ROS_INFO("X:%.2F,Y:%.2F,V:%.2F",rr(0,0),rr(1,0),rr(2,0));
      tf::Transform transform;
      geometry_msgs::TransformStamped odom_trans; 
      tf::Quaternion q;
      q.setRPY(0,0,curr_yaw);
      transform.setOrigin( tf::Vector3(curr_x, curr_y, 0.0) );
      transform.setRotation(q);
      br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "car"));
     
      //odom_trans.header.frame_id = "odom";
      //odom_trans.child_frame_id = "car";
      //odom_trans.header.stamp = ros::Time::now();
      //odom_trans.transform.translation.x = rr(0,0);
      //odom_trans.transform.translation.y = rr(1,0);
      //odom_trans.transform.translation.z = 0;
      //odom_trans.transform.rotation = odom_quat; 
      //br_.sendTransform(odom_trans);
     
    }
    */
  odom_kf_pub.publish(kf_odom_);

  geometry_msgs::Point p;
  //p.x = rf(0,0);
  //p.y = rf(1,0);
  points.header.frame_id = odom_frame_;
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
  //    ROS_INFO("X:%.2F,Y:%.2F,V:%.2F",rr(0,0),rr(1,0),rr(2,0));
}
MatrixXd pose_kf(MatrixXd xx,MatrixXd z)
{
  static ros::Time last_time, now;
  now = ros::Time::now();
  dt = (now - last_time).toSec();
  if (first)
  {
    dt = 0;
    first = false;
  }

  // double vx = speed_x.data;
  // double vy = speed_y.data;

  double vx = xx(0, 0);
  double vth = xx(1, 0);
  double yaw = X(2, 0);
  U(0, 0) = vx;
  U(1, 0) = vth;
  // F << 1,0,(double)dt * cos(speed_y.data), 0,1,(double)dt * sin(speed_y.data), 0,0,1;
  F << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 0;

  B << cos(yaw) * dt, 0,
      sin(yaw) * dt, 0,
      0, dt,
      1, 0;

  JF << 1, 0, -vx * sin(yaw) * dt, cos(yaw) * dt,
      0, 1, vx * cos(yaw) * dt, sin(yaw) * dt,
      0, 0, 1, 0,
      0, 0, 0, 1;

  // cout << "1z：\n" << z << endl;
  // cout << "1X：\n" << X << endl;
  X = F * X + B * U;
  // cout << "X:  \n" << X << endl;

  P = JF * P * JF.transpose() + Q;
  //  cout << "2P:  \n" << P << endl;
  /*
  cout << "2F:  \n" << F << endl;
  cout << "2Q:  \n" << Q << endl;
  cout << "2P:  \n" << P << endl;
  */
  /*update*/
  //  cout << "3HT:  \n" << H*P*H.transpose() << endl;
  //  cout << "3T:  \n" << R << endl;
  K = P * H.transpose() * ((H * P * H.transpose() + R).inverse());
  // cout << "3K:  \n" << K << endl;
  /*
  cout << "3P:  \n" << P << endl;
  cout << "3H:  \n" << H << endl;
  cout << "3R:  \n" << R << endl;
  cout << "3K:  \n" << K << endl;
*/
  //cout << "3HT:  \n" << H*P*H.transpose() << endl;
  //cout << "3T:  \n" << R << endl;
  X = X + K * (z - H * X);
  /*
  cout << "4X:  \n" << X << endl;
  cout << "4K:  \n" << K << endl;
  cout << "4Z:  \n" << z << endl;
  cout << "4Z-X:  \n" << z-H*X << endl;
  cout << "4K*Z-X:  \n" << K*(z-H*X) << endl;
*/
  P = P - K * H * P;
  /*
  cout << "5P:  \n" << P << endl;
  cout << "5K:  \n" << K << endl;
  cout << "5H:  \n" << H << endl;
  */
  last_time = now;
  return X;
}

void tf_callback(const ros::TimerEvent &)
{
  double x, y, yaw, ax, ay, ayaw, ux, uy, uyaw, yaw_v, siny, cosy;
  //  cout << "AP:  \n"\
       << AP << endl;
  ax = AP(0, 0);
  ay = AP(1, 0);
  ayaw = AP(2, 0);
  ux = UP(0, 0);
  uy = UP(1, 0);
  uyaw = UP(2, 0);
  x = wpa * ax + wpu * ux;
  y = wpa * ay + wpu * uy;
  //yaw = wya * ayaw + wyu * uyaw;
  siny = wya * sin(ayaw) + wyu * sin(uyaw);
  cosy = wya * cos(ayaw) + wyu * cos(uyaw);
  yaw_v = siny/cosy;
  yaw = atan(yaw_v);
  yaw = cosy<0?yaw+PI:yaw;
  yaw = yaw>PI?yaw-2*PI:yaw;
  
  if(pm2o_tf_)
  {
    tf::Quaternion q;
    q.setRPY(0,0,yaw);
    transform_.setOrigin(tf::Vector3(x, y, 0.0));
    transform_.setRotation(q);
    static tf::TransformBroadcaster br_;
    br_.sendTransform(tf::StampedTransform(transform_,  ros::Time::now(), "map", "odom"));
  }
  double vx = odom_msg_.twist.twist.linear.z;
  double vth = odom_msg_.twist.twist.angular.z;
  geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(yaw);
  
  pose_msg_.header.frame_id = "map";
  pose_msg_.child_frame_id = base_frame_;
  pose_msg_.header.stamp = ros::Time::now();
  
  pose_msg_.pose.pose.orientation = quat;
  pose_msg_.pose.pose.position.x = x;
  pose_msg_.pose.pose.position.y = y;
  pose_msg_.pose.pose.position.z = yaw;
  pose_msg_.twist.twist.linear.x = vx;
  pose_msg_.twist.twist.linear.y = 0;
  pose_msg_.twist.twist.angular.z = vth;
  
  pose_pub_.publish(pose_msg_);
  
//  ROS_INFO("YAW:%.2f,\n  YAW_V:%.2f,\n  V2Y:%.2f,", ayaw, yaw_v, test_y);
//  ROS_INFO("COS:%.2f,\n", wya * cos(ayaw) + wyu * cos(uyaw) );
//  ROS_INFO("X:%.2f, Y:%.2f, \n  AYAW:%.2f,\n  UYAW:%.2f,\n  YAW:%.2f", x, y, ayaw, uyaw, yaw);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_kf_node");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");

  private_node.param<std::string>("odom_frame", odom_frame_, std::string("odom"));
  private_node.param<std::string>("base_frame", base_frame_, std::string("base_link"));
  private_node.param<std::string>("map_frame", map_frame_, std::string("map"));
  private_node.param<bool>("pm2o", pm2o_tf_, true);
  private_node.param<double>("q", q, 0);
  private_node.param<double>("r", r, 0.1);
  private_node.param<double>("wpa", wpa, 0.5);
  private_node.param<double>("wpu", wpu, 0.5);
  private_node.param<double>("wya", wya, 0.5);
  private_node.param<double>("wyu", wyu, 0.5);
  odom_kf_pub = node.advertise<nav_msgs::Odometry>("kf_odom", 10);
  pose_pub_ = node.advertise<nav_msgs::Odometry>("luo_pos", 10);
  marker_pub = node.advertise<visualization_msgs::Marker>("kf_marker", 10);
  ros::Subscriber amcl_sub = node.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, &amcl_callback);
  ros::Subscriber u2a_sub = node.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/u2apose", 10, &u2a_callback);
  ros::Subscriber odom_sub = node.subscribe<nav_msgs::Odometry>("/odom", 10, &odom_callback);
  ros::Subscriber uwb_sub = node.subscribe<position_optimization::check_odom>("/uwb_pos", 10, &uwb_callback);
  //  ros::Subscriber vx_sub = node.subscribe<std_msgs::Float32>("/speed_x", 10, &speed_callback);
  //  ros::Subscriber vy_sub = node.subscribe<std_msgs::Float32>("/speed_y", 10, &yaw_callback);

  ros::Timer kf_timer = node.createTimer(ros::Duration(0.1), &kf_callback);
  ros::Timer tf_timer = node.createTimer(ros::Duration(0.1), &tf_callback);
  init();

  ros::spin();

  return 0;
}
