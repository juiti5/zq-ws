#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <zhongqi_driver/vehicle_info.h>
#include <check_odom.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Range.h"

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <tf/transform_broadcaster.h>
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
    double vz;
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
	void handle_sonar_lf_msg(uint8_t *buffer_data);
	void handle_sonar_lb_msg(uint8_t *buffer_data);
	void handle_sonar_rf_msg(uint8_t *buffer_data);
	void handle_sonar_rb_msg(uint8_t *buffer_data);

    void odom_check_callback(const position_optimization::check_odom::ConstPtr &msg);
    void pub_info_callback(const ros::TimerEvent &);
    void kinematic_callback(const ros::TimerEvent &);
    void odom_calculation_callback(const ros::TimerEvent &);
    void imu_callback(const sensor_msgs::Imu::ConstPtr &msg);
    void imu_2_callback(const sensor_msgs::Imu::ConstPtr &msg);
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

    position_optimization::check_odom check_msg_;
    boost::mutex check_mutex_;

    nav_msgs::Odometry odom_, odom2_;
    nav_msgs::Path odom_path;

    geometry_msgs::PointStamped position_rear_msg_,position_front_msg_;
    boost::system::error_code ec_;
    boost::asio::io_service io_service_;
    serial_port_ptr port_;

    tf::TransformBroadcaster br_;

    ros::Publisher odom_pub_, odom2_pub_, info_pub_, vx_pub_, vy_pub_;
	ros::Publisher sonar_lf_pub_, sonar_lb_pub_, sonar_rf_pub_, sonar_rb_pub_;
	ros::Time last_time_, now_;
    bool publish_tf_,check_;

    std::string port_name_;
    int baud_rate_;
    int serial_db_;

    std::string odom_frame_, base_frame_;

    int control_rate_;

    bool driving_flag_, reverse_flag_, neutral_flag_, yaw_flag_, position_flag_, first_check_;
    double wheelbase_, tread_, wheel_diameter_, maxsteer_, steer_radius_, vth_c_, vx_c_, vy_c_;
    uint8_t gear_, brake_, turn_left_lamp_, turn_right_lamp_, brake_lamp_;

    uint8_t speed_buffer[2]{}, voltage_buffer[4]{}, motor_current_buffer[2]{}, steer_buffer[1]{}, gear_buffer[1]{}, yaw_buffer[2]{}, wheel_speed_buffer[4]{}, wheel_rate_buffer[4]{}, wheel_turn_buffer[2]{};
	uint8_t sonar_lf_buffer[2]{}, sonar_lb_buffer[2]{}, sonar_rf_buffer[2]{}, sonar_rb_buffer[2]{};
	sensor_msgs::Range sonar_lf_, sonar_lb_, sonar_rf_, sonar_rb_;

	zhongqi_driver::vehicle_info current_info;
    
    sensor_msgs::Imu imu_msg_;
    boost::mutex imu_mutex_;

    sensor_msgs::Imu imu2_msg_;
    boost::mutex imu2_mutex_;

    double wheel_rate_left_, wheel_rate_right_, wheel_turn_left_, wheel_turn_right_, position_x_, position_y_, position_yaw_,imu_yaw_;

    std_msgs::Float32 vx_msg_, vy_msg_;
    car_state cs_;
    twist tt_;
  pose2d pose;
  odom od_ = {0,0,0};
};

ChassisDriver::ChassisDriver() : driving_flag_(0), reverse_flag_(0), neutral_flag_(0), yaw_flag_(1), position_flag_(1), first_check_(1), turn_left_lamp_(0), turn_right_lamp_(0), brake_lamp_(0) {}

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
  handle_sonar_lf_msg(sonar_lf_buffer);
  handle_sonar_lb_msg(sonar_lb_buffer);
  handle_sonar_rf_msg(sonar_rf_buffer);
  handle_sonar_rb_msg(sonar_rb_buffer);
//  current_info.accelerator = ackermann_msg_.drive.speed;
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

//  current_info.speed = (float)buffer_data[0] + decimal;

  current_info.speed = tt_.vx;
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
  float real_angle = buffer / 4.2;

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
/*
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
*/
 uint16_t battery;
 battery = (uint16_t)buffer_data[0];
// ROS_INFO("buffer:%02x,curr:[%d]",buffer_data[0],battery);
    current_info.battery = battery;
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
  curr_left_speed = curr_left_speed < 0.02?0:curr_left_speed;
  curr_right_speed = curr_right_speed < 0.02?0:curr_right_speed;

  if (curr_left_speed == 0 && curr_right_speed != 0)
  {
//    ROS_INFO("right!=0--:%.2f",curr_right_speed);
    curr_right_speed = 0;
  }
  if (curr_right_speed == 0 && curr_left_speed != 0)
  {
//    ROS_INFO("left!=0--:%.2f",curr_left_speed);
    curr_left_speed = 0;
  }

/*  if (curr_right_speed != 0 && curr_left_speed != 0)
  {
    ROS_INFO("l_speed:%.2f r_speed:%.2f",curr_left_speed,curr_right_speed);
  }
*/
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

//  ROS_INFO("l_speed:%.2f r_speed:%.2f",cs_.left_speed,cs_.right_speed);
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

void ChassisDriver::handle_sonar_lf_msg(uint8_t *buffer_data)
{
  uint16_t range_buffer = ((uint16_t)buffer_data[0] << 8) | buffer_data[1];
  range_buffer = static_cast<int>(range_buffer);  
//  ROS_INFO("sonar0: %d",range_buffer);
  double range = range_buffer/1000.0;

  sonar_lf_.header.stamp = ros::Time::now();
  sonar_lf_.header.frame_id = "sonar_lf_link";
  sonar_lf_.radiation_type = 0;
  sonar_lf_.field_of_view = 1.0;
  sonar_lf_.min_range = 0.2;
  sonar_lf_.max_range = 2.0;
  sonar_lf_.range = range;
  sonar_lf_pub_.publish(sonar_lf_);
}

void ChassisDriver::handle_sonar_lb_msg(uint8_t *buffer_data)
{
  uint16_t range_buffer = ((uint16_t)buffer_data[0] << 8) | buffer_data[1];
//  range_buffer = static_cast<float>(range_buffer);  
//  ROS_INFO("sonar1: %04x",range_buffer);
  double range = range_buffer/1000.0;

  sonar_lb_.header.stamp = ros::Time::now();
  sonar_lb_.header.frame_id = "sonar_lb_link";
  sonar_lb_.radiation_type = 0;
  sonar_lb_.field_of_view = 1.0;
  sonar_lb_.min_range = 0.2;
  sonar_lb_.max_range = 2.0;
  sonar_lb_.range = range;
  sonar_lb_pub_.publish(sonar_lb_);
}
void ChassisDriver::handle_sonar_rf_msg(uint8_t *buffer_data)
{
  uint16_t range_buffer = ((uint16_t)buffer_data[0] << 8) | buffer_data[1];
//  range_buffer = static_cast<float>(range_buffer);  
//  ROS_INFO("sonar2: %d",range_buffer);
  double range = range_buffer/1000.0;

  sonar_rf_.header.stamp = ros::Time::now();
  sonar_rf_.header.frame_id = "sonar_rf_link";
  sonar_rf_.radiation_type = 0;
  sonar_rf_.field_of_view = 1.0;
  sonar_rf_.min_range = 0.2;
  sonar_rf_.max_range = 2.0;
  sonar_rf_.range = range;
  sonar_rf_pub_.publish(sonar_rf_);
}
void ChassisDriver::handle_sonar_rb_msg(uint8_t *buffer_data)
{
  uint16_t range_buffer = ((uint16_t)buffer_data[0] << 8) | buffer_data[1];
//  range_buffer = static_cast<float>(range_buffer);  
//  ROS_INFO("sonar3: %04x",range_buffer);
  double range = range_buffer/1000.0;

  sonar_rb_.header.stamp = ros::Time::now();
  sonar_rb_.header.frame_id = "sonar_rb_link";
  sonar_rb_.radiation_type = 0;
  sonar_rb_.field_of_view = 1.0;
  sonar_rb_.min_range = 0.2;
  sonar_rb_.max_range = 2.0;
  sonar_rb_.range = range;
  sonar_rb_pub_.publish(sonar_rb_);
}
void ChassisDriver::odom_check_callback(const position_optimization::check_odom::ConstPtr &msg)
{
  check_mutex_.lock();
  check_msg_ = *msg.get();
  check_mutex_.unlock();
}

void ChassisDriver::imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
  imu_mutex_.lock();
  imu_msg_ = *msg.get();
  imu_mutex_.unlock();
}
  
void ChassisDriver::imu_2_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
  imu2_mutex_.lock();
  imu2_msg_ = *msg.get();
  imu2_mutex_.unlock();
}
    
void ChassisDriver::ackermann_callback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &msg)
{
  twist_mutex_.lock();
  last_twist_time_ = ros::Time::now();
  ackermann_msg_ = *msg.get();
  twist_mutex_.unlock();
}

void ChassisDriver::send_speed_callback(const ros::TimerEvent &) {
    double rotational_speed, steer, radius;
    double model_param;
    short send_speed, send_steer, send_gear, send_brake;
    double linear_speed, angular_speed;
    bool timeout = false;
    if ((ros::Time::now() - last_twist_time_).toSec() <= 1) {
        current_info.accelerator = ackermann_msg_.drive.speed;
        rotational_speed = ackermann_msg_.drive.speed;
        rotational_speed = (rotational_speed < 0) ? -rotational_speed :rotational_speed; 
        rotational_speed = (rotational_speed > 1) ? 1 :rotational_speed; 
        steer = ackermann_msg_.drive.steering_angle;
        gear_ = ackermann_msg_.drive.jerk;
        /*if rotational_speed < 0; break = 1*/
        brake_ = 0;
//        ROS_INFO("OK-time:%.2fspeed:%.5f",(ros::Time::now() - last_twist_time_).toSec(),rotational_speed);
    } else {
        timeout = true;
        current_info.accelerator = 0;
        rotational_speed = 0;
        steer = 0;
        brake_ = 1;
//	ROS_INFO("CMD_TIMEOUT!!!");
        ROS_WARN_STREAM_THROTTLE (5, "CMD_TIMEOUT!!!");

    }
//    ROS_INFO("driving_flag_:%d,reverse_flag_:%d,neutral_flag_:%d",driving_flag_,reverse_flag_,neutral_flag_);
//    ROS_INFO("gear:%d,brake:%d",gear_&3,gear_>>2);
    if((gear_&3) == 1)
    {
/*
      static uint8_t count_f;
      ROS_INFO("step:0;%d,%.2f",count_f,tt_.vx);
      if(tt_.vx < 0)
      {
        brake_ = 1;
        send_gear = 0;
      }
      else
      {
        if(tt_.vx == 0  && count_f < 3)
        {
          ROS_INFO("step:1;%d",count_f);
          brake_ = 1;
          send_gear = 0;
          count_f++;
        }
        else
        {
          brake_ = 0;
          send_gear = 1;
          reverse_flag_ = 0; 
          if(tt_.vx > 0)
          { 
            count_f = 0;
          }
          ROS_INFO("step:3;");
        }
      }
*/

      static uint8_t count_f,flag;
      first_check_ = false;
      if((reverse_flag_ == 1 || neutral_flag_ == 1 || flag == 1) && count_f <= 10)
      {
        if(neutral_flag_ == 1)
        {
          flag = 1;
        }
        brake_ = 0;  //1
        send_gear = 0;
        count_f++;

//        ROS_INFO("F1:count:%d,gear:%02x;",count_f,send_gear);
      }
      else
      {
        brake_ = 0;
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



//      rotational_speed = (rotational_speed * 60)/(PI * wheel_diameter_);
      if(rotational_speed == 0)
      {
        rotational_speed = 128;
      }
      else
      {
        rotational_speed = (abs(rotational_speed - 1) * 64); 
      }
//      rotational_speed = abs((rotational_speed * 62.13) - 128);//*1.1
 //     send_gear = 1;
    }
    else if((gear_&3) == 2)
    {
/*
      static uint8_t count_r;
      if(tt_.vx > 0)
      {
        brake_ = 1;
        send_gear = 0;
      }
      else
      {
        if(tt_.vx == 0  && count_r < 3)
        {
          brake_ = 1;
          send_gear = 0;
          count_r++;
        }
        else
        {
          brake_ = 0;      
          driving_flag_ = 0; 
          send_gear = 2;
          if(tt_.vx < 0)
          { 
            count_r = 0;
          }
       }
      }
*/

      static uint8_t count_r,flag;
      first_check_ = false;
      if((driving_flag_ == 1 || neutral_flag_ == 1 || flag ==1) && count_r <= 5)
      {
        if(neutral_flag_ == 1)
        {
          flag = 1;
        }
        brake_ = 0;  //1
        send_gear = 0;
        count_r++;

//        ROS_INFO("R1:count:%d,gear:%02x;",count_r,send_gear);
      }
      else
      {
        brake_ = 0;      
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
      if(rotational_speed == 0)
      {
        rotational_speed = 128;
      }
      else
      {
        rotational_speed = (abs(rotational_speed - 1) * 64);
      }
//      rotational_speed = (-rotational_speed * 60)/(PI * wheel_diameter_); 
//      rotational_speed = abs((-rotational_speed * 62.13) - 128);//*1.1
//      send_gear = 2;
    }
    else if((gear_&3) == 0)
    {
      static uint8_t count_n;
      if(!neutral_flag_ && count_n < 5)
      {
        brake_ = 1;   //1
        count_n++;
        send_gear = 0;
      }
      else
      {

        brake_ = 0;  
        count_n = 0;
        send_gear = 0;
      
        driving_flag_ = 0; 
        reverse_flag_ = 0; 
        neutral_flag_ = 1;
        rotational_speed = 128;
      }
    }

    if((gear_>>2) == 1 || first_check_)
    {
      brake_ = 1;
    }

    if (abs(steer)<=0.02)
    {
      steer = 128;
    }
    else
    {
    //steer = abs(steer - 1) * 128;
      steer = abs(steer>=0.532)?0.532:steer;//0.471
      steer = abs(steer<=-0.532)?-0.532:steer;
      steer = abs(steer - 0.532) * 241;//272
      steer = (steer>=255)?255:steer;
      steer = (steer<=0)?0:steer;
    }
 
//    steer = abs(steer - 1) * 106;
//    steer = (steer>=202)?202:steer;
//    ROS_INFO("_speed:%.5f,hex:%02x;",rotational_speed,send_speed);
    if(!timeout)
    {
      send_speed = static_cast<short>(rotational_speed);
      send_steer = static_cast<short>(steer);
      send_brake = brake_;
    }
    else
    {
      send_speed = static_cast<short>(128);
      send_steer = static_cast<short>(128);
      send_brake = 1;
    }

//    ROS_INFO("CMD -> steer:%.2f; speed:%.2f;",steer,rotational_speed);
//    ROS_INFO("_speed:%.5f,hex:%02x;",rotational_speed,send_speed);

    uint8_t data[12] = {0xAA, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFD};
    data[2] = send_steer;
    data[3] = send_gear;
    data[4] = send_speed;
    data[5] = send_brake;
    data[6] = turn_left_lamp_;
    data[7] = turn_right_lamp_;
    data[8] = brake_lamp_;
//    ROS_INFO("gear: [%02x],speed: [%02x],brake:[%02x]",data[3],data[4],data[5]);
    if(serial_db_ == 2 || serial_db_ == 3)
    {
      ROS_INFO("[%02x],[%02x],[%02x],[%02x],[%02x],[%02x],[%02x],[%02x],[%02x],[%02x],[%02x],[%02x]",data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8],data[9],data[10],data[11]);
    }
    if(serial_db_ == 4)
    {
      ROS_INFO("steer: [%02x],gear: [%02x],speed: [%02x],brake:[%02x]",data[2],data[3],data[4],data[5]);
    }
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
  
  tf::Quaternion quat;
  tf::quaternionMsgToTF(imu_msg_.orientation, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  imu_yaw_ = yaw;

  tt_.vth = imu_msg_.angular_velocity.z * vth_c_;
  if(abs(tt_.vth) <= 0.01)
  {
    tt_.vth = 0;
  }
  tt_.vx = (cs_.left_speed + cs_.right_speed) * vx_c_/2;
  cs_.left_speed = 0;
  cs_.right_speed = 0;
//  ROS_INFO("VX:%.2f; Vth:%.2f;",tt_.vx,tt_.vth);
  if (cs_.wheel_turn == 1)
  {
    tt_.vx = -tt_.vx;
  }
 
}

void ChassisDriver::odom_calculation_callback(const ros::TimerEvent &)
{
  static int count;
  now_ = ros::Time::now();
  double dt = (now_ - last_time_).toSec();
  tt_.vth = -tt_.vth ;
  //imu2
  double dyaw = imu2_msg_.angular_velocity.z * dt;
  static double yaw2;
  yaw2 += dyaw;

  pose.vx = tt_.vx * cos(pose.th);
  pose.vy = tt_.vx * sin(pose.th);
  pose.vz = tt_.vx;
  pose.vth = tt_.vth;
//  ROS_INFO("XX:%.2f;VX:%.2f; VY:%.2f; Th:%.2f;",tt_.vx,pose.vx,pose.vy,pose.th);

  vx_msg_.data = pose.vx;
  vx_pub_.publish(vx_msg_);
  vy_msg_.data = pose.vy;
  vy_pub_.publish(vy_msg_);
  
//  double delta_x = (pose.vx * cos(od_.odomth) - pose.vy * sin(od_.odomth)) * dt;
//  double delta_y = (pose.vx * sin(od_.odomth) + pose.vy * cos(od_.odomth)) * dt;
  double delta_x = pose.vx * dt;
  double delta_y = pose.vy * dt;
  double delta_th = tt_.vth * dt;          //vth
//  ROS_INFO("Vth:%.5f;Dt:%.2f",tt_.vth,dt);
  od_.odomx += delta_x;
  od_.odomy += delta_y;
  od_.odomth += delta_th;         //vth imu

//  ROS_INFO("VX:%.2f; VY:%.2f; Vth:%.2f; X:%.2f; Y:%.2f; TH:%.2f;",tt_.vx,tt_.vy,tt_.vth,od_.odomx,od_.odomy,od_.odomth);

  od_.odomth = angles::normalize_angle(od_.odomth);
//  ROS_INFO("YAW=odom:%.2f,uwb:%.2f",od_.odomth,check_msg_.yaw);

/*encoder and imu*/

  pose.x = od_.odomx; //encoder
  pose.y = od_.odomy; //encoder 
  pose.th = od_.odomth; 
//  ROS_INFO("IMU=X:%.2f,Y:%.2f,YAW:%.2f",pose.x,pose.y,pose.th);
//  ROS_INFO("CHECK=X:%.2f,Y:%.2f",check_msg_.x,check_msg_.y);
  if(check_)
  {
    if(check_msg_.check)
    {
      pose.x = pose.x + 0.6 * (check_msg_.x - pose.x);
      pose.y = pose.y + 0.6 * (check_msg_.y - pose.y);
      pose.th = check_msg_.yaw;      
      
      ROS_INFO("[driver_node]CHECK=X:%.2f,Y:%.2f,YAW:%.2f,OX:%.2f,OY:%.2f,OYAW:%.2f",pose.x,pose.y,pose.th,od_.odomx,od_.odomy,od_.odomth);
      od_.odomx = check_msg_.x;
      od_.odomy = check_msg_.y;
      od_.odomth = check_msg_.yaw;
    }

    else
    {
      //pose.x = pose.x + 0.4 * (check_msg_.x - pose.x);
      //pose.y = pose.y + 0.4 * (check_msg_.y - pose.y);
    }
  }

  last_time_ = now_;

//imu2
  geometry_msgs::Quaternion odom_quat2 = tf::createQuaternionMsgFromYaw(yaw2);

  odom2_.header.frame_id = odom_frame_; 
  odom2_.child_frame_id = base_frame_;
  odom2_.header.stamp = ros::Time::now();
  odom2_.pose.pose.position.x = pose.x;
  odom2_.pose.pose.position.y = pose.y;
  odom2_.pose.pose.position.z = 0;
  odom2_.pose.pose.orientation = odom_quat2;
  odom2_pub_.publish(odom2_);

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose.th);

  odom_.header.frame_id = odom_frame_; 
  odom_.child_frame_id = base_frame_;
  odom_.header.stamp = ros::Time::now();
  odom_.pose.pose.position.x = pose.x;
  odom_.pose.pose.position.y = pose.y;
  odom_.pose.pose.position.z = 0;
  odom_.pose.pose.orientation = odom_quat;
  odom_.twist.twist.linear.x = pose.vx;
  odom_.twist.twist.linear.y = pose.vy;
  odom_.twist.twist.linear.z = pose.vz;
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

    odom_trans.header.frame_id = odom_frame_;
    odom_trans.child_frame_id = base_frame_;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = pose.x;
    odom_trans.transform.translation.y = pose.y;
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = odom_quat; 

    br_.sendTransform(odom_trans);
  }

  geometry_msgs::PoseStamped this_pose_stamped;
  odom_path.header.frame_id = odom_frame_;
  odom_path.header.stamp = ros::Time::now();
  this_pose_stamped.header.frame_id = odom_frame_; //轨迹填充
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
    uint8_t state, buffer_data[35];
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
          boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[2], 30), ec_);

//          ROS_INFO("data[12]: %02x,data[13]: %02x",buffer_data[12],buffer_data[13]);
          steer_buffer[0] = buffer_data[10];
          gear_buffer[0] = buffer_data[11];
          for(int i=0;i<2;i++)
          {
            speed_buffer[i] = buffer_data[2+i];
            motor_current_buffer[i] = buffer_data[8+i];
            yaw_buffer[i] = buffer_data[12+i];
            wheel_turn_buffer[i] = buffer_data[22+i];
            sonar_lf_buffer[i] = buffer_data[24+i];
	    sonar_lb_buffer[i] = buffer_data[26+i];
	    sonar_rf_buffer[i] = buffer_data[28+i];
	    sonar_rb_buffer[i] = buffer_data[30+i];
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
          boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[32], 1), ec_);
          state = buffer_data[32] == END ? 0 : 4;
 //         ROS_INFO("buffer_data[24]: %02x,buffer_data[25]: %02x",buffer_data[28],buffer_data[29]);

          if(serial_db_ == 1 || serial_db_ == 3)
          {
            for(int i = 0; i <= 35; i++) //35
            {
              ROS_INFO("buffer_data %d:(%02x)",i,buffer_data[i]);
            }
          }

          if (state == 4) {
            ROS_DEBUG_STREAM("parse error 2 : ->" << (int) buffer_data[32]);
          }
          break;
        }
        default: {
//          ROS_INFO("default state:%d",state);
          state = 0;
          break;
        }
      } 
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
    private_node.param<bool>("check", check_, false);
    private_node.param<int>("serial_db", serial_db_, 0);
    if (init()) {
        odom_pub_ = node.advertise<nav_msgs::Odometry>("odom", 10);
        odom2_pub_ = node.advertise<nav_msgs::Odometry>("odom2", 10);
        info_pub_ = node.advertise<zhongqi_driver::vehicle_info>("/vehicle_info", 10);
        vx_pub_ = node.advertise<std_msgs::Float32>("/speed_x", 1);
        vy_pub_ = node.advertise<std_msgs::Float32>("/speed_y", 1);
        sonar_lf_pub_ = node.advertise<sensor_msgs::Range>("/sonar0", 1);
		sonar_lb_pub_ = node.advertise<sensor_msgs::Range>("/sonar1", 1);
		sonar_rf_pub_ = node.advertise<sensor_msgs::Range>("/sonar2", 1);
		sonar_rb_pub_ = node.advertise<sensor_msgs::Range>("/sonar3", 1);

        ros::Subscriber odom_check_subscribe = node.subscribe<position_optimization::check_odom>("/odom_check", 10, &ChassisDriver::odom_check_callback, this);
        ros::Subscriber ackermann_sub = node.subscribe<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 10, &ChassisDriver::ackermann_callback, this);

        ros::Subscriber imu_subscribe = node.subscribe<sensor_msgs::Imu>("imu", 10, &ChassisDriver::imu_callback, this);

        ros::Subscriber imu2_subscribe = node.subscribe<sensor_msgs::Imu>("imu_2", 10, &ChassisDriver::imu_2_callback, this);
    
        ros::Timer send_speed_timer = node.createTimer(ros::Duration(1.0 / control_rate_), &ChassisDriver::send_speed_callback, this);
        ros::Timer pub_info_timer = node.createTimer(ros::Duration(0.1), &ChassisDriver::pub_info_callback, this);
        ros::Timer kinematic_timer = node.createTimer(ros::Duration(0.1), &ChassisDriver::kinematic_callback, this);
        ros::Timer odom_calculation_timer = node.createTimer(ros::Duration(0.1), &ChassisDriver::odom_calculation_callback, this);

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

