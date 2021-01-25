#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <zhongqi_driver/vehicle_info.h>
#include "sensor_msgs/Imu.h"

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <angles/angles.h>

#define HEAD_1 0xAA
#define HEAD_2 0x55
#define END 0xFD
#define PI 3.14159

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

typedef struct
{
    float vx;
    float vy;
    float vth;
    float angle;
}twist;

typedef struct
{
    double left_speed;
    double right_speed;
    double steer;
    double angle;
    int wheel_turn;
}car_state;

typedef struct
{
    double x;
    double y;
    double th;
    double vx;
    double vy;
    double vth;
}pose2d;

typedef struct
{
  double odomx;
  double odomy;
  double odomth;
}odom;

class ChassisDriver{
private:
    bool init();
    void parse_msg();

    void handle_speed_msg(uint8_t *buffer_data);
    void handle_voltage_msg(uint8_t *buffer_data);
    void handle_motor_current_msg(uint8_t *buffer_data);
    void handle_steer_msg(uint8_t *buffer_data);
    void handle_gear_msg(uint8_t *buffer_data);
    void handle_yaw_msg(uint8_t *buffer_data);
    void handle_wheel_speed_msg(uint8_t *buffer_data);
    void handle_wheel_rate_msg(uint8_t *buffer_data);
    void handle_wheel_turn_msg(uint8_t *buffer_data);

    void pub_info_callback(const ros::TimerEvent &);
    void kinematic_callback(const ros::TimerEvent &);
    void odom_calculation_callback(const ros::TimerEvent &);
    void position_calculation_callback(const ros::TimerEvent &);
    void position_callback(const geometry_msgs::PointStamped::ConstPtr &msg);
    void position_front_callback(const geometry_msgs::PointStamped::ConstPtr &msg);
    void imu_callback(const sensor_msgs::Imu::ConstPtr &msg);

    void handle_speed_msg1(uint8_t *buffer_data);
    void ackermann_callback(const   ackermann_msgs::AckermannDriveStamped::ConstPtr &msg);
    void send_speed_callback(const ros::TimerEvent &);
    void check(uint8_t *data, size_t len, uint8_t &dest);
    std::string print_hex(uint8_t *data, int length);
public:
    ChassisDriver();

    ~ChassisDriver();

    void run();
private:


    boost::mutex twist_mutex_;
    ackermann_msgs::AckermannDriveStamped ackermann_msg_;
    ros::Time last_twist_time_;
    nav_msgs::Odometry odom_;
    nav_msgs::Path odom_path;

    boost::mutex position_mutex_;
    boost::mutex position_front_mutex_;
    geometry_msgs::PointStamped position_rear_msg_,position_front_msg_;
    boost::system::error_code ec_;
    boost::asio::io_service io_service_;
    serial_port_ptr port_;

    tf2_ros::TransformBroadcaster br_;
    geometry_msgs::TransformStamped transformStamped_;

    ros::Publisher odom_pub_, info_pub_;
    ros::Time last_time_, now_;
    bool publish_tf_;

    std::string port_name_;
    int baud_rate_;

    std::string odom_frame_, base_frame_;

    int control_rate_;

    bool driving_flag_, reverse_flag_, neutral_flag_, yaw_flag_, position_flag_;
    double wheelbase_, tread_, wheel_diameter_, maxsteer_, steer_radius_, vth_c_, vx_c_, vy_c_;
    uint8_t gear_, brake_, turn_left_lamp_, turn_right_lamp_, brake_lamp_;

    uint8_t speed_buffer[2]{}, voltage_buffer[4]{}, motor_current_buffer[2]{}, steer_buffer[1]{}, gear_buffer[1]{}, yaw_buffer[2]{}, wheel_speed_buffer[4]{}, wheel_rate_buffer[4]{}, wheel_turn_buffer[2]{};
    zhongqi_driver::vehicle_info current_info;
    
    sensor_msgs::Imu imu_msg_;
    boost::mutex imu_mutex_;
    double wheel_rate_left_, wheel_rate_right_, wheel_turn_left_, wheel_turn_right_, position_x_, position_y_, position_yaw_,position_front_x_,position_front_y_,position_rear_x_,position_rear_y_,imu_yaw_,vth_z_;

    car_state cs_;
    twist tt_;
    odom od_;
};

ChassisDriver::ChassisDriver() : driving_flag_(0), reverse_flag_(0), neutral_flag_(0), yaw_flag_(1),position_flag_(1), turn_left_lamp_(0), turn_right_lamp_(0), brake_lamp_(0) {}

ChassisDriver::~ChassisDriver() {}

bool ChassisDriver::init() {
    if (port_) {
        ROS_ERROR("error : port is already opened...");
        return false;
    }
    port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
    port_->open(port_name_, ec_);
    if (ec_) {
        ROS_INFO_STREAM("error : port_->open() failed...port_name=" << port_name_ << ", e=" << ec_.message().c_str());
        return false;
    }
    // option settings...
    port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
    port_->set_option(boost::asio::serial_port_base::character_size(8));
    port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    return true;
}

void ChassisDriver::pub_info_callback(const ros::TimerEvent &)
{
  handle_speed_msg(speed_buffer);
  handle_voltage_msg(voltage_buffer);
  handle_motor_current_msg(motor_current_buffer);
  handle_steer_msg(steer_buffer);
  handle_gear_msg(gear_buffer);
  handle_yaw_msg(yaw_buffer);
  handle_wheel_speed_msg(wheel_speed_buffer);
  handle_wheel_rate_msg(wheel_rate_buffer);
  handle_wheel_turn_msg(wheel_turn_buffer);

  info_pub_.publish(current_info);
}

void ChassisDriver::handle_speed_msg(uint8_t *buffer_data)
{
  float decimal = (float)buffer_data[1];
 
  while(1)
  {
    decimal = decimal/10;
    if(decimal < 1.0)
    {
      break;
    }
  }

  current_info.speed = (float)buffer_data[0] + decimal;

//  ROS_INFO("speed: %.2f ,decimal: %.2f",current_info.speed,decimal);
//  ROS_INFO("speed[0]: %d (%x),speed[1]: %d (%x)",buffer_data[0],buffer_data[0],buffer_data[1],buffer_data[1]);

}

void ChassisDriver::handle_voltage_msg(uint8_t *buffer_data)
{
  float decimal_12 = (float)buffer_data[1];
  float decimal_48 = (float)buffer_data[3];

  for( ;decimal_12 > 1.0; )
  {
    decimal_12 = decimal_12/10;
  }
  for( ;decimal_48 > 1.0; )
  {
    decimal_48 = decimal_48/10;
  }

  current_info.voltage_12 = (float)buffer_data[0] + decimal_12;
  current_info.voltage_48 = (float)buffer_data[2] + decimal_48;
//  ROS_INFO("voltage_12: %.2f ,decimal: %.2f",current_info.voltage_12,decimal_12);
//  ROS_INFO("speed[0]: %d (%x),speed[1]: %d (%x)",buffer_data[0],buffer_data[0],buffer_data[1],buffer_data[1]);

}

void ChassisDriver::handle_motor_current_msg(uint8_t *buffer_data)
{
  float decimal = (float)buffer_data[1];

  for( ;decimal > 1.0; )
  {
    decimal = decimal/10;
  }
  current_info.motor_current = (float)buffer_data[0] + decimal;
//  ROS_INFO("motor_current: %.2f ,decimal: %.2f",current_info.motor_current,decimal);

}

void ChassisDriver::handle_steer_msg(uint8_t *buffer_data)
{
//213
  static float curr_steer, last_steer, dif_steer;

  int buffer = -(buffer_data[0]-128);
  float real_angle = buffer / 7.0;

  current_info.steer = real_angle;

  curr_steer = real_angle * PI/180;

  dif_steer = curr_steer - last_steer;
  if(abs(dif_steer) > 0.08)
  {
    cs_.steer = last_steer;
//    ROS_INFO("ERROR:last:%.2f,curr:%.2f,diff:%.2f,cs_:%.2f ",last_steer,curr_steer,dif_steer,cs_.steer);
  }
  else
  {
    cs_.steer = curr_steer;
//    ROS_INFO("last:%.2f,curr:%.2f,diff:%.2f,cs_:%.2f ",last_steer,curr_steer,dif_steer,cs_.steer);
  }

  last_steer = cs_.steer;

//  ROS_INFO("buffer :%02x ,steer: %.2f ",buffer_data[0], cs_.steer);
//  ROS_INFO("steer[0]: %d (%x)",buffer_data[0],buffer_data[0]);
}

void ChassisDriver::handle_gear_msg(uint8_t *buffer_data)
{
  current_info.gear = buffer_data[0];
//  ROS_INFO("gear: %d",current_info.gear);
}

void ChassisDriver::handle_yaw_msg(uint8_t *buffer_data)
{
  static float last_yaw,curr_yaw,diff_yaw;

  uint16_t buffer = ((uint16_t)buffer_data[0] << 8) | buffer_data[1];
  buffer = static_cast<float>(buffer);

  float decimal = buffer/100.0;
  decimal = decimal * PI/180;
  
  curr_yaw = decimal;
  if(yaw_flag_)
  {
    last_yaw = curr_yaw;
    yaw_flag_ = false;
  }
  diff_yaw = curr_yaw - last_yaw;

  if((abs(diff_yaw) > 1.0 && abs(diff_yaw) < 4.5) || abs(diff_yaw) > 6.29)
  {
    current_info.yaw = last_yaw;
    cs_.angle = last_yaw;

//    ROS_INFO("ERROR:buffer:%04x,curr:[%.2f][%.2f];  last:%.2f;diff_yaw:%.4f",buffer,curr_yaw*180/PI,curr_yaw,last_yaw,diff_yaw);
  }
  else
  {

    current_info.yaw = curr_yaw;
    cs_.angle = curr_yaw;

//    ROS_INFO("buffer:%04x,curr:[%.2f][%.2f];  last:%.2f;diff_yaw:%.4f",buffer,curr_yaw*180/PI,curr_yaw,last_yaw,diff_yaw);
  }

//  ROS_INFO("buffer:%04x,buffer[0]:%02x,buffer[1]:%02x;yaw:%.2f",buffer,buffer_data[0], buffer_data[1],decimal);
  last_yaw = cs_.angle;
}

void ChassisDriver::handle_wheel_speed_msg(uint8_t *buffer_data) 
{
  static float curr_left_speed, curr_right_speed, diff_left_speed, diff_right_speed, last_left_speed, last_right_speed;
  uint16_t left_buffer = ((uint16_t)buffer_data[0] << 8) | buffer_data[1];
  left_buffer = static_cast<float>(left_buffer);
  uint16_t right_buffer = ((uint16_t)buffer_data[2] << 8) | buffer_data[3];
  right_buffer = static_cast<float>(right_buffer);

  curr_left_speed = left_buffer/100.0;
  curr_right_speed = right_buffer/100.0;

  diff_left_speed = curr_left_speed - last_left_speed;
  diff_right_speed = curr_right_speed - last_right_speed;
  if(abs(diff_left_speed) > 0.8)
  {
    cs_.left_speed = last_left_speed;
//    ROS_INFO("ERROR:LEFT:%04x, curr: %.2f, last: %.2f,diff:%.2f ",left_buffer, curr_left_speed, last_left_speed, diff_left_speed);
  }
  else
  {
    cs_.left_speed = curr_left_speed;
//    ROS_INFO("LEFT:%04x, curr: %.2f, last: %.2f,diff:%.2f ",left_buffer, curr_left_speed, last_left_speed, diff_left_speed);
  }

  if(abs(diff_right_speed) > 0.8)
  {
    cs_.right_speed = last_right_speed;
//    ROS_INFO("ERROR:RIGHT:%04x, curr: %.2f, last: %.2f,diff:%.2f ",right_buffer, curr_right_speed, last_right_speed, diff_right_speed);
  }
  else
  {
    cs_.right_speed = curr_right_speed;
//    ROS_INFO("RIGHT:%04x, curr: %.2f, last: %.2f,diff:%.2f ",right_buffer, curr_right_speed, last_right_speed, diff_right_speed);
  }

  last_left_speed = cs_.left_speed;
  last_right_speed = cs_.right_speed;

//  ROS_INFO("buffer_data[0]:%02x, buffer_data[1]: %02x, buffer_data: %04x(%d),speed:%.2f ",buffer_data[0], buffer_data[1],left_buffer,left_buffer,left_speed);
}

void ChassisDriver::handle_wheel_rate_msg(uint8_t *buffer_data) 
{
  uint16_t left_buffer = ((uint16_t)buffer_data[0] << 8) | buffer_data[1];
  left_buffer = static_cast<float>(left_buffer);
  uint16_t right_buffer = ((uint16_t)buffer_data[2] << 8) | buffer_data[3];
  right_buffer = static_cast<float>(right_buffer);

  float left_rate = left_buffer/100.0;
  float right_rate = right_buffer/100.0;

  wheel_rate_left_ = left_rate;
  wheel_rate_right_ = right_rate;
}

void ChassisDriver::handle_wheel_turn_msg(uint8_t *buffer_data) 
{
  wheel_turn_left_ = buffer_data[0];
  wheel_turn_right_ = buffer_data[1];
  if(wheel_turn_left_ && wheel_turn_right_)
  {
    cs_.wheel_turn = 1;
  }
  else
  {
    cs_.wheel_turn = 0;  
  }
}

void ChassisDriver::imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
  imu_mutex_.lock();
  imu_msg_ = *msg.get();
  imu_mutex_.unlock();
}

void ChassisDriver::position_callback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  position_mutex_.lock();
  position_rear_msg_ = *msg.get();
  position_mutex_.unlock();
}
      
void ChassisDriver::position_front_callback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  position_mutex_.lock();
  position_front_msg_ = *msg.get();
  position_mutex_.unlock();
}

void ChassisDriver::ackermann_callback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &msg)
{
  twist_mutex_.lock();
  last_twist_time_ = ros::Time::now();
  ackermann_msg_ = *msg.get();
  twist_mutex_.unlock();
}

void ChassisDriver::position_calculation_callback(const ros::TimerEvent &)
{
  static int count;
  static double p_rx,p_ry,init_rx,init_ry,last_rx,last_ry,diff_x,diff_y;
  static double p_fx,p_fy,init_fx,init_fy,last_fx,last_fy;
 
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

  position_rear_x_ = position_rear_msg_.point.x - init_rx;
  position_rear_y_ = position_rear_msg_.point.y - init_ry;
  position_front_x_ = position_front_msg_.point.x - init_fx;
  position_front_y_ = position_front_msg_.point.y - init_fy;
/*
  position_rear_x_ = position_rear_msg_.point.x;
  position_rear_y_ = position_rear_msg_.point.y;
  position_front_x_ = position_front_msg_.point.x;
  position_front_y_ = position_front_msg_.point.y; 
*/
  position_x_ = position_rear_x_;
  position_y_ = position_rear_y_;

  diff_x = position_front_msg_.point.x - position_rear_msg_.point.x;
  diff_y = position_front_msg_.point.y - position_rear_msg_.point.y;
  position_yaw_ = atan2(diff_y,diff_x);

//  ROS_INFO("F:%.2f[%.2f],R:%.2f[%.2f],D_X:%.2f,D_Y:%.2f,YAW:%.2f",position_front_x_,position_front_y_,position_rear_x_,position_rear_y_,diff_x,diff_y,position_yaw_);

/*IMU date*/
  
  tf::Quaternion quat;
  tf::quaternionMsgToTF(imu_msg_.orientation, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  imu_yaw_ = yaw;
  vth_z_ = imu_msg_.angular_velocity.z;
  
//  ROS_INFO("F:%.2f,R:%.2f,{%.2f}YAW:%.2f",diff_x,diff_y,diff_y/diff_x,position_yaw_);
//  ROS_INFO("X:%.2f[%.2f],Y:%.2f[%.2f],YAW:%.2f",position_x_,diff_x,position_y_,diff_y,position_yaw_);

//  ROS_INFO("position_flag_:%d[%d],real_x:%.2f,add_x:%.2f,init_x:%.2f,X:%.2f",position_flag_,count,position_msg_.point.x,p_x,init_x,position_x_);
}

void ChassisDriver::send_speed_callback(const ros::TimerEvent &) {
    double rotational_speed, steer, radius;
    double model_param;
    short send_speed, send_steer, send_gear, send_brake;
    double linear_speed, angular_speed;

    if ((ros::Time::now() - last_twist_time_).toSec() <= 1.0) {
        rotational_speed = ackermann_msg_.drive.speed;
        steer = ackermann_msg_.drive.steering_angle;
        gear_ = ackermann_msg_.drive.jerk;
        brake_ = 0;
    } else {
        rotational_speed = 0;
        steer = 0;
        brake_ = 1;
    }
//    ROS_INFO("driving_flag_:%d,reverse_flag_:%d,neutral_flag_:%d",driving_flag_,reverse_flag_,neutral_flag_);
    if(gear_ == 1)
    {
      static uint8_t count_f,flag;
      if((reverse_flag_ == 1 || neutral_flag_ == 1 || flag == 1) && count_f <= 10)
      {
        if(neutral_flag_ == 1)
        {
          flag = 1;
        }
        brake_ = 1;
        send_gear = 0;
        count_f++;

//        ROS_INFO("F1:count:%d,gear:%02x;",count_f,send_gear);
      }
      else
      {
        send_gear = 1;
        count_f = 0;
        flag = 0;
        reverse_flag_ = 0; 
      }
      if(!driving_flag_)
      {
        driving_flag_ = 1; 

        neutral_flag_ = 0;
      }



      rotational_speed = (rotational_speed * 60)/(PI * wheel_diameter_);
      rotational_speed = abs(rotational_speed * 1.1 - 128);
 //     send_gear = 1;
    }
    else if(gear_ == 2)
    {
      static uint8_t count_r,flag;

      if((driving_flag_ == 1 || neutral_flag_ == 1 || flag ==1) && count_r <= 10)
      {
        if(neutral_flag_ == 1)
        {
          flag = 1;
        }
        brake_ = 1;
        send_gear = 0;
        count_r++;

//        ROS_INFO("R1:count:%d,gear:%02x;",count_r,send_gear);
      }
      else
      {
        driving_flag_ = 0; 
        send_gear = 2;
        count_r = 0;
        flag = 0;
      }

      if(!reverse_flag_)
      {

        reverse_flag_ = 1; 
        neutral_flag_ = 0;
      }

      rotational_speed = (-rotational_speed * 60)/(PI * wheel_diameter_);
      rotational_speed = abs(rotational_speed * 1.1 - 128);
//      send_gear = 2;
    }
    else if(gear_ == 0)
    {
      if(!neutral_flag_)
      {
        brake_ = 1;
        send_gear = 0;

        driving_flag_ = 0; 
        reverse_flag_ = 0; 
        neutral_flag_ = 1;
      }

      rotational_speed = 128;
      send_gear = 0;
    }

    steer = abs(steer - 1) * 128;
    steer = (steer>=255)?255:steer;
//    steer = abs(steer - 1) * 106;
//    steer = (steer>=202)?202:steer;

    send_speed = static_cast<short>(rotational_speed);
    send_steer = static_cast<short>(steer);
    send_brake = 0;//brake_;
//    ROS_INFO("CMD -> steer:%.2f; speed:%.2f;",steer,rotational_speed);

    uint8_t data[12] = {0xAA, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFD};
    data[2] = send_steer;
    data[3] = send_gear;
    data[4] = send_speed;
    data[5] = send_brake;
    data[6] = turn_left_lamp_;
    data[7] = turn_right_lamp_;
    data[8] = brake_lamp_;
//    ROS_INFO("gear: [%02x],speed: [%02x],brake:[%02x]",data[3],data[4],data[5]);
//    ROS_INFO("[%02x],[%02x],[%02x],[%02x],[%02x],[%02x],[%02x],[%02x],[%02x],[%02x],[%02x],[%02x]",data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8],data[9],data[10],data[11]);
/*
    for(int i = 0; i < 12; i++)
    {
      ROS_INFO("buffer_data %d:(%x)",i,data[i]);
    }
*/
//    check(data, 12, data[12]);
    boost::asio::write(*port_.get(), boost::asio::buffer(data, 12), ec_);
//    ROS_DEBUG_STREAM("send -> steer: " << send_steer << "; speed: " << send_speed << "; gear: " << send_gear << "; brake: " << send_brake);
 //   ROS_INFO("send -> steer:%d; speed:%d; gear:%d; brake:  %d",send_steer,send_speed,send_gear,send_brake);
}

void ChassisDriver::kinematic_callback(const ros::TimerEvent &)
{
//  twist temp;
  tt_.vx = 0.0;
  tt_.vy = 0.0;
  tt_.vth = 0.0;

  float steer = cs_.steer;
  
  float vth_i = cs_.angle * PI / 180; 
  tt_.vx = (cs_.left_speed + cs_.right_speed) * vx_c_/2;
  if (cs_.wheel_turn == 1)
  {
    tt_.vx = -tt_.vx;
  }
//  ROS_INFO("left_speed:%.2f; right_speed:%.2f; vx:%.2f;",cs_.left_speed,cs_.right_speed,tt_.vx);
//  ROS_INFO("steer:%.2f;",steer);
  if(steer == 0)
  {
    tt_.vth = 0;
  }
  else if(steer > 0)//l
  {
    tt_.vth = tt_.vx/(wheelbase_/tan(steer) + tread_/2);
  }
  else if(steer < 0)//r
  {
    steer = -steer;
    tt_.vth = tt_.vx/(wheelbase_/tan(steer) + tread_/2);
    tt_.vth = -tt_.vth;
  }
    tt_.vth =tt_.vth * vth_c_;
//  ROS_INFO("Kinematic:Vx:%.2f; Vth:%.2f; Steer:%.2f;",tt_.vx,tt_.vth,steer);
//  return temp;
}

void ChassisDriver::odom_calculation_callback(const ros::TimerEvent &)
{
  static double last_x,curr_x,diff_x,pos_x,last_y,curr_y,diff_y,pos_y;
  curr_x = position_x_;
  curr_y = position_y_;
  diff_x = curr_x - last_x;
  diff_y = curr_y - last_y;
  if(abs(diff_x) > 3.0)
  {
    pos_x = last_x;
//    ROS_INFO("X=ERROR:%.2f;curr:%.2f;last:%.2f;diff:%.2f",pos_x,curr_x,last_x,diff_x);
  }
  else
  {
    pos_x = curr_x;
//    ROS_INFO("X:%.2f;curr:%.2f;last:%.2f;diff:%.2f",pos_x,curr_x,last_x,diff_x);
  }
  if(abs(diff_y) > 3.0)
  {
    pos_y = last_y;
//    ROS_INFO("Y=ERROR:%.2f;curr:%.2f;last:%.2f;diff:%.2f",pos_y,curr_y,last_y,diff_y);
  }
  else
  {
    pos_y = curr_y;
//    ROS_INFO("Y:%.2f;curr:%.2f;last:%.2f;diff:%.2f",pos_y,curr_y,last_y,diff_y);
  }

  pose2d pose;
  now_ = ros::Time::now();
  double dt = (now_ - last_time_).toSec();
  static double last_th;
  tt_.vth = -vth_z_ * vth_c_;
  double delta_x = (tt_.vx * cos(od_.odomth) - tt_.vy * sin(od_.odomth)) * dt;
  double delta_y = (tt_.vx * sin(od_.odomth) + tt_.vy * cos(od_.odomth)) * dt;
  double delta_th = tt_.vth * dt;          //vth
//  ROS_INFO("delta_th:%.2f; Vth:%.5f;dt:%.2f",delta_th,tt_.vth,dt);

  od_.odomx += delta_x;
  od_.odomy += delta_y;
  od_.odomth += delta_th;         //vth imu

//  od_.odomth = imu_yaw_ * vth_c_;  //yaw imu
//  od_.odomth = od_.odomth > 6.28 ? 6.28 : od_.odomth;
 //  od_.odomth = od_.odomth * vth_c_;
  od_.odomth = angles::normalize_angle(od_.odomth);

//  ROS_INFO("TH:%.2f; OTH:%.2f; Y:%.2f; last_th:%.2f",od_.odomth,cs_.angle * vth_c_,od_.odomy,last_th);
//  ROS_INFO("TH:%.2f; X:%.2f; Y:%.2f; last_th:%.2f",od_.odomth,od_.odomx,od_.odomy,last_th);
  last_th = od_.odomth;

/*encoder and imu*/
  pose.x = od_.odomx; //encoder
  pose.y = od_.odomy; //encoder 
  pose.th = od_.odomth; 

/*xinbiao*/
/*  
  pose.x = position_x_;
  pose.y = position_y_;
  pose.th = position_yaw_;
*/
//  pose.x = pos_x;
//  pose.y = pos_y;

  ROS_INFO("VTH:%.2f,YAW:%.2f",tt_.vth,pose.th);
  pose.vx = tt_.vx;
  pose.vy = tt_.vy;
  pose.vth = tt_.vth;

  last_time_ = now_;

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose.th);

  odom_.header.frame_id = "odom"; 
  odom_.child_frame_id = "base_link";
  odom_.header.stamp = ros::Time::now();
  odom_.pose.pose.position.x = pose.x;
  odom_.pose.pose.position.y = pose.y;
  odom_.pose.pose.position.z = 0;
  odom_.pose.pose.orientation = odom_quat;
  odom_.twist.twist.linear.x = pose.vx;
  odom_.twist.twist.linear.y = pose.vy;
  odom_.twist.twist.angular.z = tt_.vth;

  odom_.pose.covariance[0] = 1e-3;
  odom_.pose.covariance[7] = 1e-3;
  odom_.pose.covariance[14] = 1e6;
  odom_.pose.covariance[21] = 1e6;
  odom_.pose.covariance[28] = 1e6;
  odom_.pose.covariance[35] = 1e3;

//tf变换，里程计与baselink的
  if (publish_tf_) 
  {
    geometry_msgs::TransformStamped odom_trans; 

    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = pose.x;
    odom_trans.transform.translation.y = pose.y;
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = odom_quat; 

    br_.sendTransform(odom_trans);
  }

  geometry_msgs::PoseStamped this_pose_stamped;
  odom_path.header.frame_id = "odom";
  odom_path.header.stamp = ros::Time::now();
  this_pose_stamped.header.frame_id = "odom"; //轨迹填充
  this_pose_stamped.header.stamp = ros::Time::now();
  this_pose_stamped.pose.position.x = pose.x;
  this_pose_stamped.pose.position.y = pose.y;

  this_pose_stamped.pose.orientation.w = odom_quat.w;
  this_pose_stamped.pose.orientation.x = odom_quat.x;
  this_pose_stamped.pose.orientation.y = odom_quat.y;
  this_pose_stamped.pose.orientation.z = odom_quat.z;

  odom_path.poses.push_back(this_pose_stamped);

  odom_pub_.publish(odom_);

//  ROS_INFO("ODOM:Vx:%.2f; Vth:%.2f; Steer:%.2f;",tt_.vx,tt_.vth,tt_.angle);
}

void ChassisDriver::parse_msg()
{
    uint8_t state, buffer_data[25];
    while (1)
    {
      switch (state) {
        case 0: { // header 1
          boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[0], 1), ec_);
          state = buffer_data[0] == HEAD_1 ? 1 : 0;
          if (state == 0) {
            ROS_DEBUG_STREAM("parse error 1 : ->" << (int) buffer_data[0]);
          }
          break;
        }
        case 1: { // header 2
          boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[1], 1), ec_);
          state = buffer_data[1] == HEAD_2 ? 2 : 0;
          if (state == 0) {
            ROS_DEBUG_STREAM("parse error 2 : ->" << (int) buffer_data[1]);
          }
          break;
        }
        case 2: {
          boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[2], 22), ec_);
/*
          for(int i = 0; i <= 24; i++)
          {
            ROS_INFO("buffer_data %d:(%02x)",i,buffer_data[i]);
          }
*/
//          ROS_INFO("data[12]: %02x,data[13]: %02x",buffer_data[12],buffer_data[13]);
          steer_buffer[0] = buffer_data[10];
          gear_buffer[0] = buffer_data[11];
          for(int i=0;i<2;i++)
          {
            speed_buffer[i] = buffer_data[2+i];
            motor_current_buffer[i] = buffer_data[8+i];
            yaw_buffer[i] = buffer_data[12+i];
            wheel_turn_buffer[i] = buffer_data[22+i];
          }

          for(int i=0;i<4;i++)
          {
            voltage_buffer[i] = buffer_data[4+i];
            wheel_speed_buffer[i] = buffer_data[14+i];
            wheel_rate_buffer[i] = buffer_data[18+i];
          }
          state = 3;
          break;
        }
        case 3: {
          boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[24], 1), ec_);
          state = buffer_data[24] == END ? 0 : 4;
//          ROS_INFO("buffer_data[10]: %02x,buffer_data[15]: %02x",buffer_data[10],buffer_data[15]);
          if (state == 4) {
            ROS_DEBUG_STREAM("parse error 2 : ->" << (int) buffer_data[24]);
          }
          break;
        }
        default: {
//          ROS_INFO("default state:%d",state);
          state = 0;
          break;
        }
      } 
/*         
      boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[0], 25), ec_);

      for(int i = 0; i <= 24; i++)
      {
        ROS_INFO("buffer_data %d:(%x)",i,buffer_data[i]);
      }

      if(buffer_data[0] == HEAD_1 && buffer_data[1] == HEAD_2 && buffer_data[24] == END)
      {
        steer_buffer[0] = buffer_data[10];
        gear_buffer[0] = buffer_data[11];
        ROS_INFO("buffer_data 0:(%x)",steer_buffer[0]);
        for(int i=0;i<2;i++)
        {
          speed_buffer[i] = buffer_data[2+i];
          motor_current_buffer[i] = buffer_data[8+i];
          yaw_buffer[i] = buffer_data[12+i];
          wheel_turn_buffer[i] = buffer_data[22+i];
        }

        for(int i=0;i<4;i++)
        {
          voltage_buffer[i] = buffer_data[4+i];
          wheel_speed_buffer[i] = buffer_data[14+i];
          wheel_rate_buffer[i] = buffer_data[18+i];
        }
      }
*/
    } 
}

void ChassisDriver::check(uint8_t *data, size_t len, uint8_t &dest) {
    dest = 0x00;
    for (int i = 0; i < len; i++) {
        dest = dest ^ *(data + i);
    }
}

std::string ChassisDriver::print_hex(uint8_t *data, int length) {
    std::string output = "";
    static char hex[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
    for (int i = 0; i < length; i++) {
        output += hex[(*(data + i) >> 4) & 0xf] + hex[*(data + i) & 0xf] + " ";
    }
    return output;
}

void ChassisDriver::run() {
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");

    private_node.param<std::string>("port_name", port_name_, std::string("/dev/ttyUSB0"));

    private_node.param<std::string>("odom_frame", odom_frame_, std::string("odom"));
    private_node.param<std::string>("base_frame", base_frame_, std::string("base_link"));

    private_node.param<double>("vth_c", vth_c_, 1);
    private_node.param<double>("vx_c", vx_c_, 1);
    private_node.param<double>("vy_c", vy_c_, 1);

    private_node.param<int>("control_rate", control_rate_, 10);
    private_node.param<int>("baud_rate", baud_rate_, 115200);
    private_node.param<double>("wheelbase", wheelbase_, 1.07);
    private_node.param<double>("tread", tread_, 0.685);
    private_node.param<double>("wheel_diameter", wheel_diameter_, 0.37);
    private_node.param<double>("maxsteer", maxsteer_, 1);
    private_node.param<double>("steer_radius", steer_radius_, 3.5);
    private_node.param<bool>("publish_tf", publish_tf_, true);

    if (init()) {
        odom_pub_ = node.advertise<nav_msgs::Odometry>("wheel_odom", 10);
        info_pub_ = node.advertise<zhongqi_driver::vehicle_info>("vehicle_info", 10);

        ros::Subscriber ackermann_sub = node.subscribe<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 10, &ChassisDriver::ackermann_callback, this);
        ros::Subscriber position_subscribe = node.subscribe<geometry_msgs::PointStamped>("location_pos", 10, &ChassisDriver::position_callback, this);
        ros::Subscriber position_front_subscribe = node.subscribe<geometry_msgs::PointStamped>("location_pos_2", 10, &ChassisDriver::position_front_callback, this);
        ros::Subscriber imu_subscribe = node.subscribe<sensor_msgs::Imu>("imu", 10, &ChassisDriver::imu_callback, this);
              
        ros::Timer send_speed_timer = node.createTimer(ros::Duration(1.0 / control_rate_), &ChassisDriver::send_speed_callback, this);
        ros::Timer pub_info_timer = node.createTimer(ros::Duration(0.5), &ChassisDriver::pub_info_callback, this);
        ros::Timer kinematic_timer = node.createTimer(ros::Duration(0.1), &ChassisDriver::kinematic_callback, this);
        ros::Timer odom_calculation_timer = node.createTimer(ros::Duration(0.3), &ChassisDriver::odom_calculation_callback, this);
        ros::Timer position_timer = node.createTimer(ros::Duration(0.3), &ChassisDriver::position_calculation_callback, this);
        boost::thread parse_thread(boost::bind(&ChassisDriver::parse_msg, this));

        ros::spin();

        return;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "zhongqi_driver");
    ChassisDriver driver;
    driver.run();
    return 0;
}

