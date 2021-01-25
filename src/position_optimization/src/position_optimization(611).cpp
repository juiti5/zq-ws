#include <ros/ros.h>
//#include <time.h>
#include <geometry_msgs/TransformStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "sensor_msgs/Imu.h"
#include <position_optimization/check_odom.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <angles/angles.h>
#include <std_msgs/Float32.h>

#include <eigen3/Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include<vector>
using namespace std;
//using namespace ros;
//using namespace Eigen;
using Eigen::MatrixXd;

#define PI 3.14159

typedef struct
{
    double x;
    double y;
    double z;
    double th;
    double vx;
    double vy;
    double vz;
    double vth;
    geometry_msgs::Quaternion th_q;
//    tf::Quaternion th_q;
}pose2d;
typedef struct
{
  double odomx;
  double odomy;
  double odomth;
}odom;
typedef struct
{
  double x;
  double y;
  double th;
  double time;
}uwb_odom;
typedef struct
{
  double ox;
  double oy;
  double x;
  double y;
  double yaw;
}u2o_trans;

class Optimization{
private:
    bool init();
    
    void odom_calculation_callback(const ros::TimerEvent &);
    void position_calculation_callback(const ros::TimerEvent &);
    void check_position_callback(const ros::TimerEvent &);
    
    void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
    void kf_odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
    void position_callback(const geometry_msgs::PointStamped::ConstPtr &msg);
    void position_front_callback(const geometry_msgs::PointStamped::ConstPtr &msg);
    void imu_callback(const sensor_msgs::Imu::ConstPtr &msg);

public:
    Optimization();

    ~Optimization();

    void run();
private:

    boost::mutex odom_mutex_;
    boost::mutex position_rear_mutex_;
    boost::mutex position_front_mutex_;
    boost::mutex imu_mutex_;
    
    nav_msgs::Odometry odom_;
    nav_msgs::Odometry odom_msg_;
    nav_msgs::Odometry kf_odom_msg_;
    nav_msgs::Odometry odom_uwb_;
    nav_msgs::Path odom_path;
    nav_msgs::Odometry pose_msg_;
    
    geometry_msgs::PointStamped position_rear_msg_,position_front_msg_;
    sensor_msgs::Imu imu_msg_;
    position_optimization::check_odom check_msg_; 
    
    tf::TransformBroadcaster br_;
    tf::TransformBroadcaster map_odom_br_;
    tf::TransformListener listener;
    geometry_msgs::TransformStamped transformStamped_;
    
//    ros::Publisher odom_pub_;
    ros::Publisher check_odom_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher data_pub_;
    bool publish_tf_,optimization_,trans_ok_;
    std::string odom_frame_, base_frame_, map_frame_;
    int control_rate_;



    double position_x_, position_y_, position_yaw_,imu_yaw_,vth_z_;
    double  real_position_front_x_,real_position_front_y_,real_position_rear_x_,real_position_rear_y_;

    bool real_position_,position_flag_;

    uwb_odom uod_;
    u2o_trans trans;
/*    
    tf::Transform map_to_odom_transform_;
    Eigen::Matrix3d rotation_;
    Eigen::Vector3d trans_;
*/    

    Eigen::Matrix3d origin_rear_;
    Eigen::Matrix3d origin_front_;
    Eigen::Matrix3d origin_zero_;
    Eigen::Matrix3d transfrom_;
    Eigen::Matrix3d rotation_z_;
    Eigen::Matrix3d rotation_x_;
    Eigen::Matrix3d result_rear_;
    Eigen::Matrix3d result_front_;
    Eigen::Matrix3d result_zero_;    
};

Optimization::Optimization() :position_flag_(true),real_position_(false),trans_ok_(false) 
{
//    map_to_odom_transform_ = tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0, 0, 0));
}

Optimization::~Optimization() {}

bool Optimization::init() {
   
   return true;
}

void Optimization::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  odom_mutex_.lock();
  odom_msg_ = *msg.get();
  odom_mutex_.unlock();
}

void Optimization::kf_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  kf_odom_msg_ = *msg.get();
  //double p1yaw=0;

  double pox= kf_odom_msg_.pose.pose.position.x;
  double poy= kf_odom_msg_.pose.pose.position.y;

/*
  double tx = trans.ox;
  double ty = trans.oy;
  double ro = trans.yaw;
  Eigen::MatrixXd origin;
  Eigen::MatrixXd rotation_z;
  Eigen::MatrixXd rotation_x;
  Eigen::MatrixXd transfrom ;
  Eigen::MatrixXd result;

  origin = MatrixXd(3, 1);
  rotation_z = MatrixXd(3, 3);
  rotation_x = MatrixXd(3, 3);
  transfrom  = MatrixXd(3, 1);
  result = MatrixXd(3, 1);
            
  origin << pox,poy,0;   
  //origin << 2,1,0;  
  rotation_z << cos(ro), sin(ro), 0, -sin(ro), cos(ro), 0, 0, 0, 1;
  rotation_x << 1, 0, 0, 0, cos(PI), sin(PI), 0, -sin(PI), cos(PI);
  transfrom << tx, ty, 0;      
  
 // cout<<" origin " << origin << endl;
  cout<<" tx " << tx <<" ty " << ty <<" ro " << ro << endl;
  //cout<<" rotation_z " << rotation_z << endl;
  //cout<<" transfrom " << transfrom << endl;
  result =  rotation_x * rotation_z * (origin - transfrom);
  cout<<" out " << result <<endl;
  double mx = result(0,0);
  double my = result(1,0);
*/



  //way 2
  double x = kf_odom_msg_.pose.pose.orientation.x;
  double y = kf_odom_msg_.pose.pose.orientation.y;
  double z = kf_odom_msg_.pose.pose.orientation.z;
  double w = kf_odom_msg_.pose.pose.orientation.w;

  Eigen::Vector3d  po= Eigen::Vector3d(pox,poy, 0);
  Eigen::Quaterniond qo(w,x,y,z); 

  double tmoyaw = -trans.yaw;
  double tmox = trans.ox;
  double tmoy = trans.oy;
  Eigen::Vector3d ea(PI, 0, tmoyaw);
  //cout << ea[0] << "  " << ea[2] << ro << endl;
  Eigen::Quaterniond qmo;
  qmo = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitX()) * 
                Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) * 
                Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond qmon = qmo.normalized();
  Eigen::Vector3d  tmo= Eigen::Vector3d(tmox,tmoy, 0); 

  Eigen::Quaterniond qm=qo*qmo;
  Eigen::Vector3d pm=qmon.inverse() * (po - tmo);
 // cout<<"PMX"<<pm[0]<<"PMY"<<  pm[1] <<endl;
  //cout<<"QMX"<<qm.x()<<"QMW"<<  qm.w() <<endl;
 // cout<<"t2 eular angle "<<(180/M_PI)*qm.matrix().eulerAngles(0,1,2)<<endl;
  double mx = pm[0];
  double my = pm[1]; 
  
  static double curr_x, curr_y, last_x, last_y, diff_x, diff_y;
  static double curr_yaw, last_yaw, diff_yaw, real_yaw;
  curr_x = pm[0];
  curr_y = pm[1];
  diff_x = curr_x - last_x;
  diff_y = curr_y - last_y;
  last_x = curr_x;
  last_y = curr_y;

  double kf_yaw = atan2(diff_y,diff_x);
  curr_yaw = kf_yaw;
  diff_yaw = abs(curr_yaw - last_yaw);
  diff_yaw = diff_yaw > PI ? diff_yaw - 2 * PI : diff_yaw;
  
  double total_weight = 50.0, odom_weight = 48.0,uwb_weight = 2.0;
/*
  tf::Quaternion o_quat;
  tf::quaternionMsgToTF(odom_msg_.pose.pose.orientation, o_quat);
  double o_roll, o_pitch, o_yaw;
  tf::Matrix3x3(o_quat).getRPY(o_roll, o_pitch, o_yaw);
  double myaw = o_yaw + trans.yaw;
  //myaw = myaw > PI ? 2 * PI  - myaw : myaw;
  geometry_msgs::Quaternion quat_o = tf::createQuaternionMsgFromYaw(myaw);
 
  Eigen::Quaterniond q_o;
  q_o.x()=quat_o.x; //odom_msg_.pose.pose.orientation.x;
  q_o.y()=quat_o.y; //odom_msg_.pose.pose.orientation.y;
  q_o.z()=quat_o.z; //odom_msg_.pose.pose.orientation.z;
  q_o.w()=quat_o.w; //odom_msg_.pose.pose.orientation.w;

  Eigen::Vector3d ea1(PI, PI,  PI);
  
  Eigen::Quaterniond q_mo;
  q_mo = Eigen::AngleAxisd(ea1[0], Eigen::Vector3d::UnitX()) * 
                Eigen::AngleAxisd(ea1[1], Eigen::Vector3d::UnitY()) * 
                Eigen::AngleAxisd(ea1[2], Eigen::Vector3d::UnitZ());
  
  Eigen::Quaterniond m_quat =  q_o * q_mo;

geometry_msgs::Quaternion map_q;
map_q.x = m_quat.x();
map_q.y = m_quat.y();
map_q.z = m_quat.z();
map_q.w = m_quat.w();
*/



  static ros::Time last_time, now;
  now = ros::Time::now();
  double dt = (now - last_time).toSec();

  double vth = odom_msg_.twist.twist.angular.z;
  double vx = odom_msg_.twist.twist.linear.z;
  double dvth = diff_yaw/dt;
  
  
//  if((abs(abs(vth) - abs(dvth)) > 0.2) && vth > 0.01)
  if((abs(diff_yaw) > 0.2) && abs(vth) > 0.01)
  {
    real_yaw = last_yaw;//*8 + curr_yaw*2)/10;
    curr_yaw = real_yaw;
    ROS_INFO("ERROR>>=VTH:%.5F,DVTH:%.2F,DY:%.2F",vth,dvth,diff_yaw);
  }
  else if((abs(diff_yaw) > 0.5) && abs(vx) <= 0.01)
  {
    real_yaw = last_yaw;//*8 + curr_yaw*2)/10;
    curr_yaw = real_yaw;
    ROS_INFO("ERROR000=VTH:%.2F,DVTH:%.2F,DY:%.2F",vth,dvth,diff_yaw);
  }
  else
  {
     real_yaw = curr_yaw;
     ROS_INFO("OKVTH>>=VTH:%.2F,DVTH:%.2F,DY:%.2F",vth,dvth,diff_yaw);
  }
  ROS_INFO("REAL>>=TH:%.2F",real_yaw);
  last_time = now;


  if(abs(abs(vth) - abs(dvth)) > 100.03)
  //if(diff_yaw > 0.2)
  {
      //o_yaw = o_yaw<0?o_yaw+2*PI:o_yaw;
      //last_yaw = last_yaw<0?last_yaw+2*PI:last_yaw;
    if(curr_yaw > 0 && last_yaw < 0)
    {
      double dy = curr_yaw + (-last_yaw);
      double w;
      if(dy > PI)
      {
        w = (2*PI - dy)/total_weight;
      } 
      else if(dy < PI)
      {
        w = dy/total_weight;
      }
        real_yaw = last_yaw + uwb_weight*w;
        real_yaw = real_yaw > PI ? 2 * PI  - real_yaw : real_yaw;
    }
    else if(curr_yaw < 0 && last_yaw > 0)
    {
      double dy = last_yaw + (-curr_yaw);
      double w;
      if(dy > PI)
      {
        w = -(2*PI - dy)/total_weight;
      } 
      else if(dy < PI)
      {
        w = -dy/total_weight;
      }
        real_yaw = last_yaw + uwb_weight*w;
        real_yaw = real_yaw > PI ? 2 * PI  - real_yaw : real_yaw;
    }
    else 
    {
      real_yaw = (last_yaw*odom_weight + curr_yaw*uwb_weight)/total_weight;
    }
    //real_yaw = (curr_yaw*15 + last_yaw*5)/20;//(curr_yaw*10 + last_yaw*9 + curr_yaw*1)/30;
    //real_yaw = real_yaw > PI ? real_yaw - 2 * PI : real_yaw;
    ROS_INFO("BIG=CY:%.2F,LY:%.2F,DY:%.2F",curr_yaw,last_yaw,diff_yaw);
    curr_yaw = real_yaw;
  }
  else
  {
    real_yaw = curr_yaw;
   // ROS_INFO("OK=CY:%.2F,LY:%.2F,DY:%.2F",curr_yaw,last_yaw,diff_yaw);
  }
  

  last_yaw = curr_yaw;
  
  //geometry_msgs::Quaternion quat;
  geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(real_yaw);
  //quat.x = real_yaw;
  // kf_odom_.pose.pose.orientation = odom_quat;


  /*
  quat.x = kf_odom_msg_.pose.pose.orientation.x;     //qm.x();
  quat.y = kf_odom_msg_.pose.pose.orientation.y;     //qm.y();
  quat.z = kf_odom_msg_.pose.pose.orientation.z;     //qm.z();
  quat.w = kf_odom_msg_.pose.pose.orientation.w;     //qm.w();
*/
  pose_msg_.header.frame_id = "tag_map"; 
  pose_msg_.child_frame_id = base_frame_;
  pose_msg_.header.stamp = ros::Time::now();

  pose_msg_.pose.pose.orientation = quat;
  pose_msg_.pose.pose.position.x = mx;
  pose_msg_.pose.pose.position.y = my;
 // pose_msg_.pose.pose.position.x = uod_.x;
 // pose_msg_.pose.pose.position.y = uod_.y;
  pose_msg_.pose.pose.position.z = real_yaw;

 
}

void Optimization::imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
  imu_mutex_.lock();
  imu_msg_ = *msg.get();
  imu_mutex_.unlock();
}

void Optimization::position_callback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  position_rear_mutex_.lock();
  position_rear_msg_ = *msg.get();
  position_rear_mutex_.unlock();
}
      
void Optimization::position_front_callback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  position_front_mutex_.lock();
  position_front_msg_ = *msg.get();
  position_front_mutex_.unlock();
}

void Optimization::check_position_callback(const ros::TimerEvent &)
{

  double ox_f, oy_f, ox_r, oy_r, tx, ty, ro, odom_x, odom_y;
  static double diff_x,diff_y,odom_yaw;
  
  /*
  tf::StampedTransform local_transform;
  
  geometry_msgs::PointStamped rear_point;
  geometry_msgs::PointStamped odom_rear_point;
  rear_point.header.stamp = ros::Time();
  rear_point.header.frame_id = map_frame_;
  rear_point.point.x = real_position_rear_x_;
  rear_point.point.y = real_position_rear_y_;
  rear_point.point.z = 0;
  
  geometry_msgs::PointStamped front_point;
  geometry_msgs::PointStamped odom_front_point;
  front_point.header.stamp = ros::Time();
  front_point.header.frame_id = map_frame_;
  front_point.point.x = real_position_front_x_;
  front_point.point.y = real_position_front_y_;
  rear_point.point.z = 0;
  try{
  listener.transformPoint(odom_frame_,rear_point,odom_rear_point); 
  listener.transformPoint(odom_frame_,front_point,odom_front_point);
  }

  catch (tf::TransformException &ex) {
   ROS_ERROR("%s",ex.what());
   ros::Duration(1.0).sleep();
  }
  diff_x = odom_front_point.point.x - odom_rear_point.point.x;
  diff_y = odom_front_point.point.y - odom_rear_point.point.y;
*/

  ox_f = real_position_front_x_;
  oy_f = real_position_front_y_;
  ox_r = real_position_rear_x_;
  oy_r = real_position_rear_y_;
  tx = trans.x;
  ty = trans.y;
  ro = -trans.yaw;
  
  origin_front_ << ox_f,0,0, oy_f,1,0, 0,0,1;
  origin_rear_ << ox_r,0,0, oy_r,1,0, 0,0,1;
  origin_zero_ << 0,0,0, 0,1,0, 0,0,1;                     
  rotation_z_ << cos(ro), -sin(ro), 0, sin(ro), cos(ro), 0, 0, 0, 1;
  rotation_x_ << 1, 0, 0, 0, cos(PI), -sin(PI), 0, sin(PI), cos(PI);
  transfrom_ << tx,0,0, ty,1,0, 0,0,1;         
  
  result_front_ = rotation_x_ * rotation_z_ * (origin_front_ - transfrom_);
  result_rear_ = rotation_x_ * rotation_z_ * (origin_rear_ - transfrom_);
  result_zero_ = rotation_x_ * rotation_z_ * (origin_zero_ - transfrom_);
  
  odom_x = result_rear_(0,0);
  odom_y = result_rear_(1,0);
  
  trans.ox = result_zero_(0,0);
  trans.oy = result_zero_(1,0);

  diff_x = result_front_(0,0) - result_rear_(0,0);
  diff_y = result_front_(1,0) - result_rear_(1,0);
  odom_yaw = atan2(diff_y,diff_x);
/*************************************************************************/
static double uy, oy, cy, ly, wu, wo;
uy = odom_yaw<0?odom_yaw+2*PI:odom_yaw;

tf::Quaternion quat;
tf::quaternionMsgToTF(odom_msg_.pose.pose.orientation, quat);
double roll, pitch, yaw;
tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
oy = yaw<0?yaw+2*PI:yaw;

cy = uy;
double dth = abs(cy - ly);
dth = dth > PI ? 2 * PI - dth : dth;
wo = dth *10;
wo = wo>20?20:wo;
wu = 20 - wo;
double fy = (oy*(wo+30)+uy*wu)/50.0;
fy = fy > PI ? fy - 2 * PI : fy;
//ROS_INFO("WO:%.2f, WU:%.2f, DY:%.2f,UY:%.2f, FY:%.2f",wo+30,wu,dth,odom_yaw,fy);
ly = cy;
/****************************************************************************************/
//ROS_INFO("UY:%.2f, OY:%.2f, DY:%.2f",uy,oy,dth);
//  odom_yaw = angles::normalize_angle(odom_yaw);
/*
  cout << "Origen：\n" << origin_rear_ << endl;
  cout << "Rotation：\n" << rotation_z_ << endl;
  cout << "Transfrom：\n" << transfrom_ << endl;
*/
//  cout << "Result：\n" << result_rear_ << endl;
 

//  ROS_INFO("RESULT=X:%.2f,Y:%.2f,YAW:%.2f",odom_x,odom_y,odom_yaw);

//  ROS_INFO("OX:%.2f, OY:%.2f, TX:%.2f, TY:%.2f",ox,oy,trans.x, trans.y);
//  ROS_INFO("UWB=X:%.2f, Y:%.2f, YAW:%.2f",odom_rear_point.point.x,odom_rear_point.point.y,odom_yaw);

	if(real_position_)
	{
      uod_.x = odom_x;
      uod_.y = odom_y;
      uod_.th = fy;
      uod_.time = ros::Time::now().toSec();
      check_msg_.xx = result_front_(0,0);
      check_msg_.yy = result_front_(1,0);
	/*
		pose.x = position_x_;
		pose.y = position_y_;
	*/
	
	}
  
}

void Optimization::position_calculation_callback(const ros::TimerEvent &)
{
  static int count;
  static double p_rx,p_ry,p_yaw,init_rx,init_ry,diff_x,diff_y;
  static double p_fx,p_fy,init_fx,init_fy,last_fx,last_fy;
  double position_front_x,position_front_y,position_rear_x,position_rear_y,distance;
  static double start_x,start_y,start_th;
  static double curr_yaw,last_yaw,diff_yaw;
  static double  dt_r0,dt_r1,dt_r2,cu_rx,cu_ry,l0_rx,l0_ry,diff_r0,diff_r1;
  static double  dt_f0,dt_f1,dt_f2,cu_fx,cu_fy,l0_fx,l0_fy,diff_f0,diff_f1;
  bool dist,rate,zero,rvth;
  static ros::Time last_time, now;
  std_msgs::Float32 dist_msg;
  /*
  if(position_flag_ && count < 10)
  {
    p_rx += position_rear_msg_.point.x;
    p_ry += position_rear_msg_.point.y;
    p_fx += position_front_msg_.point.x;
    p_fy += position_front_msg_.point.y;
    count++;
  }
  else
  {
    position_flag_ = false;
  }
  init_rx = p_rx/10.0;
  init_ry = p_ry/10.0;
  init_fx = p_fx/10.0;
  init_fy = p_fy/10.0;
  */
  
/*
  position_rear_x = position_rear_msg_.point.x - init_rx;
  position_rear_y = position_rear_msg_.point.y - init_ry;
  position_front_x = position_front_msg_.point.x - init_fx;
  position_front_y = position_front_msg_.point.y - init_fy;
*/  
  
///*
  position_rear_x = position_rear_msg_.point.x;
  position_rear_y = position_rear_msg_.point.y;
  position_front_x = position_front_msg_.point.x;
  position_front_y = position_front_msg_.point.y; 
//*/

  diff_x = position_front_msg_.point.x - position_rear_msg_.point.x;
  diff_y = position_front_msg_.point.y - position_rear_msg_.point.y;
  distance = sqrt(pow(diff_x,2) + pow(diff_y,2));

  cu_rx = position_rear_x;
  cu_ry = position_rear_y; 
  cu_fx = position_front_x;
  cu_fy = position_front_y;
    
//  if(distance < 0.75)
//  {
    dt_r0 = sqrt(pow((cu_rx - l0_rx),2) + pow((cu_ry - l0_ry),2));
    dt_f0 = sqrt(pow((cu_fx - l0_fx),2) + pow((cu_fy - l0_fy),2));
//  ROS_INFO("dt_r0:%.2f,dt_r1:%.2f,dt_r2:%.2f",dt_r0,dt_r1,dt_r2);

    diff_r0 = abs(dt_r0 - dt_r1);
    diff_r1 = abs(dt_r1 - dt_r2);
    diff_f0 = abs(dt_f0 - dt_f1);
    diff_f1 = abs(dt_f1 - dt_f2);
//    ROS_INFO("diff_r0:%.2f,diff_r1:%.2f,dd:%.5f",diff_r0,diff_r1,diff_r0 - diff_r1);
//  } 
  dt_r2 = dt_r1;
  dt_r1 = dt_r0;
  l0_rx = cu_rx;
  l0_ry = cu_ry;
  dt_f2 = dt_f1;
  dt_f1 = dt_f0;
  l0_fx = cu_fx;
  l0_fy = cu_fy; 
    
  now = ros::Time::now();
  curr_yaw = atan2(diff_y,diff_x);
  diff_yaw = abs(curr_yaw - last_yaw);
  diff_yaw = diff_yaw > PI ? 2 * PI - diff_yaw : diff_yaw;
  double vth = abs(odom_msg_.twist.twist.angular.z);
  double vx = abs(odom_msg_.twist.twist.linear.z);
  double dt = (now - last_time).toSec();
  last_time = now;
  
  dist_msg.data = diff_yaw/dt;
  data_pub_.publish(dist_msg);
  if((diff_yaw/dt) > (vth + 0.05) || (diff_yaw/dt) < (vth - 0.05))
  {
//    ROS_INFO("Rvth!=D:[%.5f]DV:[%.5f]",diff_yaw,diff_yaw/dt-vth);
    rvth = false;
  }
  else if(vth == 0 && diff_yaw/dt > 0.03)
  {
    rvth = false;
  }
  else
  {
//    ROS_INFO("OK_Rvth_OK=D:[%.5f]DV:[%.5f]",diff_yaw,diff_yaw/dt-vth);
    rvth = true;  
  }
//      position_yaw_ = (curr_yaw + last_yaw)/2.0;
//      ROS_INFO("==YAW==CURR:%.2f,LAST:%.2f,DIFF:%.2f",curr_yaw,last_yaw,diff_yaw);
      
      last_yaw = curr_yaw;
  if(vx > 0.01 && (distance > 1.50 || distance < 0.70))
  {
  /*
  	static int i = 1;
  	static double j,k;
  	
  	j+=distance;
  	k=j/i;
  	i++;
  */
    //ROS_INFO("VX>>0;Distance![%.2f]",distance);
    dist = false;
  }
  else if(vx < 0.01 && (distance > 0.75 || distance < 0.55))
  {
  /*
    static int l = 1;
  	static double m,n;
  	
  	m+=distance;
  	n=m/l;
  	l++;
  */
    //ROS_INFO("VX=0;Distance[%.2f]",distance);
    dist = false;  
  }
  else
  {
//    ROS_INFO("Distance_OK[%.2f]",distance);
    dist = true;  
  }

//  if(dist && (abs(diff_r0 - diff_r1) > 0.03 || abs(diff_f0 - diff_f1) > 0.03))
  if(/*dt_r0/dt > (vx + 0.03) || dt_r0/dt < (vx - 0.03) ||*/ dt_f0/dt > (vx + 0.15) || dt_f0/dt < (vx - 0.15))
  {
    rate = false;
//    ROS_INFO("Rate!=F:[%.5f]R:[%.5f]DF[%.5f]DR[%.5f]",dt_f0/dt,dt_r0/dt,(dt_f0/dt)-vx,(dt_r0/dt)-vx);
    
//    ROS_INFO("Rate![%.5f]{[%.5f][%.5f]}",vx,dt_f0/dt,(dt_f0/dt)-vx);
    
  }
  else
  {
//    ROS_INFO("OK_Rate_OK=F:[%.5f]R:[%.5f]DF[%.5f]DR[%.5f]",dt_f0/dt,dt_r0/dt,(dt_f0/dt)-vx,(dt_r0/dt)-vx);
    rate = true;  
  }

  if(0 == position_rear_x && 0 == position_rear_y && 0 == position_front_x && 0 == position_front_y)
  {
    zero = false;
  }
  else
  {
    zero = true;  
  }


//  ROS_INFO("distance:%.2f",distance);
  rvth = dist = rate = zero = true;
  if((!rvth || !dist || !rate || !zero)/* && !trans_ok_*/)
  {
    real_position_ = false;
    //ROS_INFO("False pose![%d][%d][%d][%d][%d]",rvth,dist,rate,zero,trans_ok_);
  }
  else
  {
      //ROS_INFO("True pose!");
//    if((position_rear_x >= -1.0 && position_rear_x <= 8.0) && (position_rear_y >= -1.0 && position_rear_y <= 10.0))
//    {
      position_yaw_ = atan2(diff_y,diff_x);

      if(position_flag_ && count < 10 && distance > 0)
      {
        p_rx += position_rear_x;
        p_ry += position_rear_y;
        p_yaw += position_yaw_;
        count++;
      }
      else
      {
        if(distance > 0)
        {
          position_flag_ = false;      
        }
      }
      
      if(count >= 10)
      {
        start_x = p_rx/10.0;
        start_y = p_ry/10.0;
        start_th = p_yaw/10.0;
        trans_ok_ = true; 
//        start_th = position_yaw_;

//        ROS_INFO("==TRANS==X:%.2f,Y:%.2f,YAW:%.2f",start_x,start_y,start_th);
      }
      
      trans.x = start_x;
      trans.y = start_y;
      trans.yaw = start_th;
      
      real_position_ = true;
      real_position_rear_x_ = position_rear_msg_.point.x;
      real_position_rear_y_ = position_rear_msg_.point.y;
      real_position_front_x_ = position_front_msg_.point.x;
      real_position_front_y_ = position_front_msg_.point.y;
      position_x_ = real_position_rear_x_;
      position_y_ = real_position_rear_y_;
/*      
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(position_yaw_);
         
      pose_msg_.header.stamp = ros::Time::now();
      pose_msg_.pose.pose.orientation = odom_quat;
      pose_msg_.pose.pose.position.x = position_x_;
      pose_msg_.pose.pose.position.y = position_y_;
      pose_msg_.pose.pose.position.z = 0;
*/

//      ROS_INFO("RX:%.2f,RY:%.2f",position_x_,position_y_);
//      ROS_INFO("TRANS=X:%.2f,Y:%.2f,YAW:%.2f",start_x,start_y,start_th);

//    }
  }
//  ROS_INFO("F:%.2f[%.2f],R:%.2f[%.2f],D_X:%.2f,D_Y:%.2f,YAW:%.2f",position_front_x_,position_front_y_,position_rear_x_,position_rear_y_,diff_x,diff_y,position_yaw_);
//   ROS_INFO("PX:%.2f,PY:%.2f,S_X:%.2f,S_Y:%.2f,PYAW:%.2f,SYAW:%.2f",position_x_,position_y_,start_x,start_y,position_yaw_,start_th);

 //tf map_odom public
    if(publish_tf_)
    {
      geometry_msgs::Quaternion map_quat = tf::createQuaternionMsgFromYaw(start_th);
     // geometry_msgs::Quaternion map_quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch,start_th);
      geometry_msgs::TransformStamped map_trans; 

      map_trans.header.frame_id = map_frame_;
      map_trans.child_frame_id = odom_frame_;
      map_trans.header.stamp = ros::Time::now();
      map_trans.transform.translation.x = start_x;
      map_trans.transform.translation.y = start_y;
      map_trans.transform.translation.z = 0;
      map_trans.transform.rotation = map_quat; 
      map_odom_br_.sendTransform(map_trans);
    }

/*IMU date*/
/*  
  tf::Quaternion quat;
  tf::quaternionMsgToTF(imu_msg_.orientation, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  imu_yaw_ = yaw;
  vth_z_ = imu_msg_.angular_velocity.z;
*/
  
//  ROS_INFO("F:%.2f,R:%.2f,{%.2f}YAW:%.2f",diff_x,diff_y,diff_y/diff_x,position_yaw_);
//  ROS_INFO("X:%.2f[%.2f],Y:%.2f[%.2f],YAW:%.2f",position_x_,diff_x,position_y_,diff_y,position_yaw_);

//  ROS_INFO("position_flag_:%d[%d],real_x:%.2f,add_x:%.2f,init_x:%.2f,X:%.2f",position_flag_,count,position_msg_.point.x,p_x,init_x,position_x_);
}



void Optimization::odom_calculation_callback(const ros::TimerEvent &)
{
  pose2d pose;
  static odom od_;
  bool check = true;
  static double last_uwbx,curr_uwbx,diff_uwbx,pos_uwbx,last_uwby,distance,curr_uwby,diff_uwby,pos_uwby;
  
  static ros::Time last_time, now;
  
  pose.x = odom_msg_.pose.pose.position.x;
  pose.y = odom_msg_.pose.pose.position.y;
  pose.z = odom_msg_.pose.pose.position.z;
  pose.vx = odom_msg_.twist.twist.linear.x;
  pose.vy = odom_msg_.twist.twist.linear.y;
  pose.vz = odom_msg_.twist.twist.linear.z;
  pose.vth = odom_msg_.twist.twist.angular.z;
  pose.th_q = odom_msg_.pose.pose.orientation;
  
  tf::Quaternion quat;
  tf::quaternionMsgToTF(pose.th_q, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  pose.th = yaw;
  
  if(optimization_ && trans_ok_)
  {
    curr_uwbx = uod_.x;
    curr_uwby = uod_.y;
    diff_uwbx = curr_uwbx - last_uwbx;
    diff_uwby = curr_uwby - last_uwby;
 
    if(real_position_)
    {   
      distance = sqrt(pow(diff_uwbx,2) + pow(diff_uwby,2));

      if(distance > (abs(pose.vx) + 0.01) || distance < (abs(pose.vx) - 0.01))
      {
//        ROS_INFO("distance[%.5f] > vx[%.5f]",distance,abs(pose.vx));
        check = false;    
      }
      else
      {
        check = true;
//        ROS_INFO("OK=distance[%.5f] > vx[%.5f]",distance,abs(pose.vx));
      }
     
      if(abs(pose.vx) <= 0.01 && distance > 0.001)
      {
//        ROS_INFO("Zero!");
        check = false;    
      }
      else
      {
        check = true;
//        ROS_INFO("NO_Zero!");
      }
    }
    else
    {
      check = false; 
    }
//    ROS_INFO("STATY:: R[%d] || C[%d]=VX:%.2f,DT:%.2f,DX:%.2f,[%.2f,%.2f]",real_position_,check,abs(pose.vx),distance,diff_uwbx,curr_uwbx,curr_uwby);
//    ROS_INFO("STATY:: R[%d]C[%d]",real_position_,check);
//    ROS_INFO("OX:%.2f,OY:%.2f,YAW:%.2f",uod_.x,uod_.y,uod_.th);
    check = true;
    if(check)
    {
//      ROS_INFO("RX:%.2f,RY:%.2f,YAW:%.2f",curr_uwbx,curr_uwby,uod_.th);
      double dth = abs(uod_.th - pose.th);
      dth = dth > PI ? 2 * PI - dth : dth;
//      ROS_INFO("CHECK_YAW=[%.2f][%.2f]",uod_.th,pose.th);
      now = ros::Time::now();
      if((distance > 1.0 || dth > 0.3) && (now - last_time).toSec() < 1.0)
      {
        check_msg_.check = false;
        //ROS_INFO("CHECK_ERROR=[%.2f][%.2f][%.2f]",distance,dth,(now - last_time).toSec());  
     /*  
       if((now - last_time).toSec() > 3.0)
       {
         check_msg_.check = true;
         ROS_INFO("TIME>1[%.2f]==OYAW:%.2f,CYAW:%.2f",(now - last_time).toSec(),pose.th,uod_.th);
       }
       else
       {
         ROS_INFO("CHECK_ERROR=[%.2f][%.2f]",distance,dth);       
       }
*/
      }
      else
      {
        check_msg_.check = true;
        last_time = ros::Time::now();
        
//        ROS_INFO("CHECK_OK=X:%.2f,Y:%.2f,YAW:%.2f",curr_uwbx,curr_uwby,uod_.th);
      }
      pos_uwbx = curr_uwbx;
      pos_uwby = curr_uwby;
    
      pose.x = pos_uwbx;
      pose.y = pos_uwby;
      
      check_msg_.x = pose.x;
      check_msg_.y = pose.y;
      check_msg_.yaw = uod_.th;
      /*
      if((now - last_time).toSec() > 1.0)
      {
        check_msg_.yaw = pose.th;
        ROS_INFO("FIRST==OYAW:%.2f,CYAW:%.2f",pose.th,uod_.th);
      }  
      */    
    }
    else
    {
      pose.x = pose.x;
      pose.y = pose.y;
      
      pos_uwbx = last_uwbx;
      pos_uwby = last_uwby;
            
      check_msg_.check = false;
      check_msg_.x = pose.x;
      check_msg_.y = pose.y;
    }
  
    last_uwbx = curr_uwbx;
    last_uwby = curr_uwby;
  }
  
  pose_msg_.twist.twist.angular.x = uod_.x;
  pose_msg_.twist.twist.angular.y = uod_.y;  
  pose_msg_.twist.twist.linear.x = pose.vx;
  pose_msg_.twist.twist.linear.y = pose.vy;
  pose_msg_.twist.twist.angular.z = pose.vth;
  pose_pub_.publish(pose_msg_);
  check_odom_pub_.publish(check_msg_);
  /*
  static bool i;
  if(i)
  {
    ROS_INFO("IPOSE==%d",i);  
    pose_pub_.publish(pose_msg_);
    i = false;
  }
  else
  {
    ROS_INFO("ICHECK==%d",i);
    check_odom_pub_.publish(check_msg_);
    i = true;
  }
  */
  
}

void Optimization::run() {
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");

    private_node.param<std::string>("odom_frame", odom_frame_, std::string("odom"));
    private_node.param<std::string>("base_frame", base_frame_, std::string("base_link"));
    private_node.param<std::string>("map_frame", map_frame_, std::string("map"));

    private_node.param<int>("control_rate", control_rate_, 3.0);
    private_node.param<bool>("publish_tf", publish_tf_, true);
    private_node.param<bool>("optimization", optimization_, true);
    if (init()) {
//        odom_pub_ = node.advertise<nav_msgs::Odometry>("odom", 10);
        check_odom_pub_ = node.advertise<position_optimization::check_odom>("odom_check", 10);
        pose_pub_ = node.advertise<nav_msgs::Odometry>("carpos", 10);
        data_pub_ = node.advertise<std_msgs::Float32>("data", 1);
    
        ros::Subscriber odom_sub = node.subscribe<nav_msgs::Odometry>("/odom", 10, &Optimization::odom_callback, this);
        ros::Subscriber kf_odom_sub = node.subscribe<nav_msgs::Odometry>("/kf_odom", 10, &Optimization::kf_odom_callback, this);
        ros::Subscriber position_subscribe = node.subscribe<geometry_msgs::PointStamped>("location_pos", 10, &Optimization::position_callback, this);
        ros::Subscriber position_front_subscribe = node.subscribe<geometry_msgs::PointStamped>("location_pos_2", 10, &Optimization::position_front_callback, this);
        ros::Subscriber imu_subscribe = node.subscribe<sensor_msgs::Imu>("imu", 10, &Optimization::imu_callback, this);
              
        ros::Timer odom_calculation_timer = node.createTimer(ros::Duration(1.0/control_rate_), &Optimization::odom_calculation_callback, this);
        ros::Timer position_timer = node.createTimer(ros::Duration(1.0/control_rate_), &Optimization::position_calculation_callback, this);
        ros::Timer check_position_timer = node.createTimer(ros::Duration(1.0/control_rate_), &Optimization::check_position_callback, this);

        ros::spin();

        return;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "optimization_node");
    Optimization op;
    op.run();
    return 0;
}

