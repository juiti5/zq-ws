#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <string>
#include <vector>
#include <math.h>
#include <dynamic_reconfigure/server.h>
#include <driver_sim/drivercfgConfig.h>

#include <ackermann_msgs/AckermannDriveStamped.h>

#include "tf/transform_datatypes.h"
#include <boost/bind.hpp>

using namespace std;

class SimDriver
{
public:
  SimDriver();
  ~SimDriver();
  void run();
  void Config_Callback(driver_sim::drivercfgConfig &config, uint32_t level);

  double p_, i_, d_;
private:
  void ackermann_callback(const   ackermann_msgs::AckermannDriveStamped::ConstPtr &msg);
  void twist_callback(const geometry_msgs::Twist::ConstPtr &msg);
  void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
  void calculation_callback(const ros::TimerEvent&);
  float calculation_pid(float goal, float current);
  float limit(float data, float max);
  float convert_trans_rot_vel_to_steering_angle(float v, float omega, float wheelbase);

  ackermann_msgs::AckermannDriveStamped ackermann_msg_;
  geometry_msgs::Twist current_twist_;
  double current_yaw_;
  float  current_vx_;
  float max_steer_;
  nav_msgs::Odometry current_odom_;
//  ros::Time current_time_;
  ros::Publisher rr_pub_, rl_pub_, fr_pub_, fl_pub_;
  ros::Timer calculation_timer_;
  double ratio_vel_cmd_,  ratio_steer_cmd_, ratio_out_;

  double goal_, current_, error_, last_error_, past_error_, sum_error_, result_;

  float speed_, steering_, wheelbase_;
};

SimDriver::SimDriver() : wheelbase_(2.85), ratio_vel_cmd_(10.0), ratio_steer_cmd_(0.3), max_steer_(0.5),  ratio_out_(2),current_yaw_(0),current_vx_(0), p_(10.0), i_(20.0), d_(1.0) {}
SimDriver::~SimDriver(){}
float SimDriver::limit(float data, float max)
{
  float temp;
  if(-max >= data)
  {
    temp = -max;
  }
  else if(max <= data)
  {
    temp = max;
  }
  else
  {
    temp = data;
  }
  return temp;
}
float SimDriver::calculation_pid(float goal,float current)
{

  float out;
  goal_ = goal;
  current_ = current;
  error_ = goal_ - current_;
  if(0.1 >= error_ && -0.1 <= error_)
  {
//    ROS_INFO("error = 0");
//    ROS_INFO("goal:[%.2f],current:[%.2f],error:[%.2f],out:[%.2f]", goal_,current_,error_,result_);
    return 0;
  }

  sum_error_ += error_;
//  result_ = p_ * error_ + i_ * sum_error_ + d_ * (error_ - last_error_);
  result_ = p_ * (error_ - last_error_) + i_ * (error_) + d_ * (error_ - 2 * last_error_ + past_error_);

  past_error_ = last_error_;
  last_error_ = error_;
  
//  out = (current + result_) * ratio_out_;
  out = result_ * ratio_out_;
//  ROS_INFO("YAW:[%.2f], goal:[%.2f],current:[%.2f],error:[%.2f],out:[%.2f],p:[%.2f],i:[%.2f],d:[%.2f]", current_yaw_, goal_,current_,error_,result_,p_,i_,d_);
  return out;
}
float SimDriver::convert_trans_rot_vel_to_steering_angle(float v, float omega, float wheelbase)
{
  if (omega == 0 || v == 0)
  {
     return 0;
  }
   
  float radius = v / omega;
  return atan(wheelbase / radius);
}
void SimDriver::calculation_callback(const ros::TimerEvent&)
{
  float goal_vx, goal_vth, vx, vy;
  std_msgs::Float64 vel_cmd, steer_cmd;
  float current_vx1,current_vx2;
/*
  goal_vx = current_twist_.linear.x;
  goal_vth = current_twist_.angular.z;
*/

  goal_vx = ackermann_msg_.drive.speed;
  goal_vth = ackermann_msg_.drive.steering_angle;
// ROS_INFO("CMD_VEL=vel_cmd:[%.2f],steer_cmd:[%.2f]", linear_speed,angular_speed);

  vx = current_odom_.twist.twist.linear.x;
  vy = current_odom_.twist.twist.linear.y;
  //current_vx = sqrt(pow(vx,2) + pow(vy,2));
  current_vx1 = vy / sin(current_yaw_);
  current_vx2 = vx / cos(current_yaw_);
  current_vx_ = (current_vx1 + current_vx2) / 2;
//  ROS_INFO("YAW:[%.2f],CMD_VEL=current_vx1:[%.2f],current_vx2:[%.2f],current_vx:[%.2f]",current_yaw_, current_vx1,current_vx2,current_vx_);

  speed_ = calculation_pid(goal_vx,current_vx_) * ratio_vel_cmd_;
  vel_cmd.data = speed_;


  steer_cmd.data = goal_vth * ratio_steer_cmd_;
  steer_cmd.data = limit(steer_cmd.data, max_steer_);

/*
  steering_ = convert_trans_rot_vel_to_steering_angle(goal_vx, goal_vth, wheelbase_);
  steer_cmd.data = limit(steering_, max_steer_);
*/
/*
  if(goal_vth = 0)
  {
    if(goal_vx != 0)
    {
      speed_ = calculation_pid(goal_vx,current_vx_) * ratio_vel_cmd_;
      vel_cmd.data = speed_;

      steer_cmd.data = 0; 
    }
    else 
    {
      vel_cmd.data = 0;
      steer_cmd.data = 0;     
    }
  }
  else
  {
    if(goal_vx = 0)
    {
      speed_ = calculation_pid(goal_vth * 10,current_vx_) * ratio_vel_cmd_;
      vel_cmd.data = speed_;
 
      steer_cmd.data = limit(goal_vth, max_steer_); 
    }
    else 
    {
      speed_ = calculation_pid(goal_vx, current_vx_) * ratio_vel_cmd_;
      vel_cmd.data = speed_;
 
      steer_cmd.data = limit(goal_vth, max_steer_);     
    } 
  }
*/
  ROS_INFO("PUB=vel_cmd:[%.2f],steer_cmd:[%.2f]", vel_cmd.data,steer_cmd.data);

  rr_pub_.publish(vel_cmd);
  rl_pub_.publish(vel_cmd);
  fr_pub_.publish(steer_cmd);
  fl_pub_.publish(steer_cmd);
}

void SimDriver::twist_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
  current_twist_ = *msg.get();
}

void SimDriver::ackermann_callback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &msg)
{
  ackermann_msg_ = *msg.get();
}

void SimDriver::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  current_odom_ = *msg.get();

  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
  double roll, pitch, yaw;//定义存储r\p\y的容器
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
  current_yaw_ = yaw;
//  ROS_INFO("YAW: %.2f ", yaw);
}

void SimDriver::Config_Callback(driver_sim::drivercfgConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure Request: %.2f %.2f %.2f", config.p, config.i, config.d);

  p_ = config.p;
  i_ = config.i;
  d_ = config.d;

}
void SimDriver::run()
{
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");
  private_node.param<float>("wheelbase", wheelbase_, 2.85);
  private_node.param<double>("ratio_vel_cmd", ratio_vel_cmd_, 10.0);
  private_node.param<double>("ratio_steer_cmd", ratio_steer_cmd_, 0.3);
  private_node.param<float>("max_steer", max_steer_, 0.5);
  private_node.param<double>("ratio_out", ratio_out_, 2.0);
  private_node.param<double>("p", p_, 35.0);
  private_node.param<double>("i", i_, 70.0);
  private_node.param<double>("d", d_, 0.0);

  ros::Subscriber cmd_sub = node.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &SimDriver::twist_callback, this);
  ros::Subscriber ackermann_sub = node.subscribe<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 10, &SimDriver::ackermann_callback, this);
  ros::Subscriber odom_sub = node.subscribe<nav_msgs::Odometry>("/odom", 10, &SimDriver::odom_callback, this);

  rr_pub_ = node.advertise<std_msgs::Float64
>("parkingdemo/rr_Wheel_effort_controller/command", 10); 
  rl_pub_ = node.advertise<std_msgs::Float64
>("parkingdemo/rl_Wheel_effort_controller/command", 10); 
  fr_pub_ = node.advertise<std_msgs::Float64
>("parkingdemo/fr_Steer_position_controller/command", 10);
  fl_pub_ = node.advertise<std_msgs::Float64
>("parkingdemo/fl_Steer_position_controller/command", 10); 	
  calculation_timer_ = node.createTimer(ros::Duration(1.0/10), &SimDriver::calculation_callback, this);
/*
  SimDriver sim_driver1;
  dynamic_reconfigure::Server<driver_sim::drivercfgConfig> server;
  dynamic_reconfigure::Server<driver_sim::drivercfgConfig>::CallbackType f;
  f = boost::bind(&SimDriver::Config_Callback, &sim_driver1, _1, _2);
  server.setCallback(f);
*/
  ros::spin();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "driver_sim");
  ROS_INFO("Runing SimDriver node");
  SimDriver sim_driver;

  dynamic_reconfigure::Server<driver_sim::drivercfgConfig> server;
  dynamic_reconfigure::Server<driver_sim::drivercfgConfig>::CallbackType f;
  f = boost::bind(&SimDriver::Config_Callback, &sim_driver, _1, _2);
  server.setCallback(f);

  sim_driver.run();

  return 0;
}

