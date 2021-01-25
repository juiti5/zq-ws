#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <zhongqi_driver/vehicle_info.h>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>


#define HEAD_1 0xAA
#define HEAD_2 0x55
#define END 0xFD
#define SPEED_MSG_ID 0x01
#define BATTERY_MSG_ID 0x02
#define CURRENT_MSG_ID 0X07
#define VOLTAGE_MSG_ID 0x08
#define ERROR_MSG_ID 0xFF

#define PI 3.14159

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

class ChassisDriver{
private:
    bool init();
    void parse_msg();
    void distribute_msg(uint8_t msg_type, uint8_t *buffer_data);
    void handle_speed_msg(uint8_t *buffer_data);
    void handle_voltage_msg(uint8_t *buffer_data);
    void handle_motor_current_msg(uint8_t *buffer_data);
    void handle_steer_msg(uint8_t *buffer_data);
    void handle_gear_msg(uint8_t *buffer_data);
    void handle_yaw_msg(uint8_t *buffer_data);
    void pub_info_callback(const ros::TimerEvent &);

//    void twist_callback(const geometry_msgs::Twist::ConstPtr &msg);
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
    bool parse_flag_;
    uint8_t msg_seq_;

    boost::mutex twist_mutex_;
    geometry_msgs::Twist current_twist_;
    ackermann_msgs::AckermannDriveStamped ackermann_msg_;
    ros::Time last_twist_time_;
    nav_msgs::Odometry odom_;

    boost::system::error_code ec_;
    boost::asio::io_service io_service_;
    serial_port_ptr port_;
    boost::mutex mutex_;

    tf2_ros::TransformBroadcaster br_;
    geometry_msgs::TransformStamped transformStamped_;

    ros::Publisher speed_pub_, voltage_pub_, current_pub_, odom_pub_, info_pub_;
    ros::Time last_time_, now_;
    bool publish_tf_;

    std::string port_name_;
    int baud_rate_;

    std::string odom_frame_, base_frame_;

    int control_rate_, sensor_rate_;

    double maximum_encoding_;
    double pulse_per_cycle_, encoder_resolution_, reduction_ratio_, pid_rate_;

    bool driving_flag_, reverse_flag_, neutral_flag_;
    double wheelbase_, tread_, wheel_diameter_, maxsteer_, steer_radius_;
    uint8_t gear_, brake_;

    uint8_t speed_buffer[2], voltage_buffer[4], motor_current_buffer[2], steer_buffer[1], gear_buffer[1], yaw_buffer[2];

    zhongqi_driver::vehicle_info current_info;
//  double total_dis_;

    bool start_flag_;
    double delta_time_;
    double accumulation_x_, accumulation_y_, accumulation_th_;
    int cur_left_, cur_right_, rev_left_, rev_right_, delta_left_, delta_right_;

};

ChassisDriver::ChassisDriver() : driving_flag_(0), reverse_flag_(0), neutral_flag_(0) {}

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
/*
void ChassisDriver::twist_callback(const geometry_msgs::Twist::ConstPtr &msg) {
    twist_mutex_.lock();
    last_twist_time_ = ros::Time::now();
    current_twist_ = *msg.get();
    twist_mutex_.unlock();
}
*/
void ChassisDriver::pub_info_callback(const ros::TimerEvent &)
{
  handle_speed_msg(speed_buffer);
  handle_voltage_msg(voltage_buffer);
  handle_motor_current_msg(motor_current_buffer);
  handle_steer_msg(steer_buffer);
  handle_gear_msg(gear_buffer);
  handle_yaw_msg(yaw_buffer);

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
  float decimal12 = (float)buffer_data[1];
  float decimal48 = (float)buffer_data[3];

  for( ; decimal12 > 1.0; )
  {
    decimal12 = decimal12/10;
  }
  for( ; decimal48 > 1.0; )
  {
    decimal48 = decimal48/10;
  }

  current_info.voltage_12 = (float)buffer_data[0] + decimal12;
  current_info.voltage_48 = (float)buffer_data[2] + decimal48;
//  ROS_INFO("voltage12: %.2f ,decimal: %.2f",current_info.voltage12,decimal12);
//  ROS_INFO("speed[0]: %d (%x),speed[1]: %d (%x)",buffer_data[0],buffer_data[0],buffer_data[1],buffer_data[1]);


}

void ChassisDriver::handle_motor_current_msg(uint8_t *buffer_data)
{
  float decimal = (float)buffer_data[1];

  for( ; decimal > 1.0; )
  {
    decimal = decimal/10;
  }
  current_info.motor_current = (float)buffer_data[0] + decimal;
//  ROS_INFO("motor_current: %.2f ,decimal: %.2f",current_info.motor_current,decimal);

}

void ChassisDriver::handle_steer_msg(uint8_t *buffer_data)
{
/*
  float decimal = buffer_data[0];

  for( ; decimal > 1.0; )
  {
    decimal = decimal/10;
  }
*/
  current_info.steer = buffer_data[0];
//  ROS_INFO("motor_current: %.2f ,decimal: %.2f",current_info.steer,decimal);
//  ROS_INFO("steer[0]: %d (%x)",buffer_data[0],buffer_data[0]);
}

void ChassisDriver::handle_gear_msg(uint8_t *buffer_data)
{
  current_info.gear = buffer_data[0];
//  ROS_INFO("gear: %d",current_info.gear);
}

void ChassisDriver::handle_yaw_msg(uint8_t *buffer_data)
{
  uint16_t buffer = ((uint16_t)buffer_data[0] << 8) | buffer_data[1];
  buffer = static_cast<float>(buffer);
  float decimal = buffer/100;
/*
  for( ; decimal > 1.0; )
  {
    decimal = decimal/10;
  }
*/
  current_info.yaw = decimal;
//  ROS_INFO("buffer_data[0]:%04x, buffer_data[1]: %04x ",buffer_data[0], buffer_data[1]);
//  ROS_INFO("buffer:%04x, yaw: %.2f ",buffer, current_info.yaw);

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

    if(gear_ == 1)
    {
      if(!driving_flag_)
      {
/*
        uint8_t driving_gear[12] = {0xAA, 0x55, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFD};
        boost::asio::write(*port_.get(), boost::asio::buffer(driving_gear, 12), ec_);
*/
        brake_ = 1;
        send_gear = 1;

        driving_flag_ = 1; 
        reverse_flag_ = 0; 
        neutral_flag_ = 0;
      }

      rotational_speed = (rotational_speed * 60)/(PI * wheel_diameter_);
//      radius = rotational_speed/current_twist_.angular.z;
//      steer = atan(wheelbase_/radius);
      rotational_speed = abs(rotational_speed * 1.1 - 128);
      send_gear = 1;
    }
    else if(gear_ == 2)
    {
      if(!reverse_flag_)
      {
/*
        uint8_t reverse_gear[12] = {0xAA, 0x55, 0x00, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFD};
        boost::asio::write(*port_.get(), boost::asio::buffer(reverse_gear, 12), ec_);
*/
        brake_ = 1;
        send_gear = 2;

        driving_flag_ = 0; 
        reverse_flag_ = 1; 
        neutral_flag_ = 0;
      }

      rotational_speed = (-rotational_speed * 60)/(PI * wheel_diameter_);
      rotational_speed = abs(rotational_speed * 1.1 - 128);
      send_gear = 2;
    }
    else if(gear_ == 0)
    {
      if(!neutral_flag_)
      {
/*
        uint8_t neutral_gear[12] = {0xAA, 0x55, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFD};
        boost::asio::write(*port_.get(), boost::asio::buffer(neutral_gear, 12), ec_);
*/
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

    send_speed = static_cast<short>(rotational_speed);
    send_steer = static_cast<short>(steer);
    send_brake = brake_;
//    ROS_INFO("CMD -> steer:%.2f; speed:%.2f;",steer,rotational_speed);

    uint8_t data[12] = {0xAA, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFD};
    data[2] = send_steer;
    data[3] = send_gear;
    data[4] = send_speed;
    data[5] = send_brake;

//    check(data, 12, data[12]);
    boost::asio::write(*port_.get(), boost::asio::buffer(data, 12), ec_);
//    ROS_DEBUG_STREAM("send -> steer: " << send_steer << "; speed: " << send_speed << "; gear: " << send_gear << "; brake: " << send_brake);
 //   ROS_INFO("send -> steer:%d; speed:%d; gear:%d; brake:  %d",send_steer,send_speed,send_gear,send_brake);
}

void ChassisDriver::handle_speed_msg1(uint8_t *buffer_data) {
    rev_left_ = buffer_data[5] * 256 + buffer_data[6];
    rev_right_ = buffer_data[7] * 256 + buffer_data[8];

//    cal_pulse(cur_left_, rev_left_, delta_left_);
//    cal_pulse(cur_right_, rev_right_, delta_right_);

    ROS_DEBUG_STREAM("receive -> left: " << delta_left_ << "(" << rev_left_ << ")" << "; right: " << delta_right_ << "(" << rev_right_ << ")");

    now_ = ros::Time::now();
    if (start_flag_) {
        accumulation_x_ = accumulation_y_ = accumulation_th_ = 0.0;
        last_time_ = now_;
        start_flag_ = false;
        return;
    }
    delta_time_ = (now_ - last_time_).toSec();
    if (delta_time_ >= (0.5 / control_rate_)) {
/*        double model_param;
        if (delta_right_ <= delta_left_) {
            model_param = model_param_cw_;
        } else {
            model_param = model_param_acw_;
        }
*/
        double delta_theta = (delta_right_ - delta_left_) / (pulse_per_cycle_ * pid_rate_ * 1);
        double v_theta = delta_theta / delta_time_;

        double delta_dis = (delta_right_ + delta_left_) / (pulse_per_cycle_ * pid_rate_ * 2.0);
        double v_dis = delta_dis / delta_time_;

        double delta_x, delta_y;
        if (delta_theta == 0) {
            delta_x = delta_dis;
            delta_y = 0.0;
        } else {
            delta_x = delta_dis * (sin(delta_theta) / delta_theta);
            delta_y = delta_dis * ((1 - cos(delta_theta)) / delta_theta);
        }

        accumulation_x_ += (cos(accumulation_th_) * delta_x - sin(accumulation_th_) * delta_y);
        accumulation_y_ += (sin(accumulation_th_) * delta_x + cos(accumulation_th_) * delta_y);
        accumulation_th_ += delta_theta;

        tf2::Quaternion q;
        q.setRPY(0, 0, accumulation_th_);

        if (publish_tf_) {
            transformStamped_.header.stamp = ros::Time::now();
            transformStamped_.header.frame_id = odom_frame_;
            transformStamped_.child_frame_id = base_frame_;
            transformStamped_.transform.translation.x = accumulation_x_;
            transformStamped_.transform.translation.y = accumulation_y_;
            transformStamped_.transform.translation.z = 0.0;

            transformStamped_.transform.rotation.x = q.x();
            transformStamped_.transform.rotation.y = q.y();
            transformStamped_.transform.rotation.z = q.z();
            transformStamped_.transform.rotation.w = q.w();

            br_.sendTransform(transformStamped_);
        }

        odom_.header.frame_id = odom_frame_;
        odom_.child_frame_id = base_frame_;
        odom_.header.stamp = now_;
        odom_.pose.pose.position.x = accumulation_x_;
        odom_.pose.pose.position.y = accumulation_y_;
        odom_.pose.pose.position.z = 0;
        odom_.pose.pose.orientation.x = q.getX();
        odom_.pose.pose.orientation.y = q.getY();
        odom_.pose.pose.orientation.z = q.getZ();
        odom_.pose.pose.orientation.w = q.getW();
        odom_.twist.twist.linear.x = v_dis;
        odom_.twist.twist.linear.y = 0;
        odom_.twist.twist.angular.z = v_theta;

        odom_pub_.publish(odom_);

        ROS_DEBUG_STREAM("accumulation_x: " << accumulation_x_ << "; accumulation_y: " << accumulation_y_ << "; accumulation_th: " << accumulation_th_);
    }
    last_time_ = now_;
}

void ChassisDriver::parse_msg()
{
    uint8_t state, buffer_data[15]={0xAA, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFD};
    while (1)
    {
      boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[0], 15), ec_);
//      for(int i = 0; i <= 14; i++)
//     {
//        ROS_INFO("buffer_data %d:(%x)",i,buffer_data[i]);
//      }

      if(buffer_data[0] == HEAD_1 && buffer_data[1] == HEAD_2 && buffer_data[14] == END)
      {
        steer_buffer[0] = buffer_data[10];
        gear_buffer[0] = buffer_data[11];
       
        for(int i=0;i<2;i++)
        {
          speed_buffer[i] = buffer_data[2+i];
          motor_current_buffer[i] = buffer_data[8+i];
          yaw_buffer[i] = buffer_data[12+i];
        }

        for(int i=0;i<4;i++)
        {
          voltage_buffer[i] = buffer_data[4+i];
        }
      }
    }

/*  
    parse_flag_ = true;
    while (parse_flag_)
    {
        switch (state)
        {
            case 0: { // header 1
//                        check_num = 0x00;
                        boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[0], 1), ec_);
                        state = buffer_data[0] == HEAD_1 ? 1 : 0;
                        //                ROS_INFO("case=0,state=%d,buffer_data=%x",state,buffer_data[0]);

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
            case 2: { // speed
                        boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[2], 2), ec_);
                        for(int i=0;i<2;i++)
                        {
                            speed_buffer[i] = buffer_data[2+i];
                        }
                        state = 3;
                        break;
                    }
            case 3: { // 12V
                        boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[4], 4), ec_);
                        for(int i=0;i<4;i++)
                        {
                            voltage_buffer[i] = buffer_data[4+i];
                        }

                        state = 4;
                        break;
                    }
            case 4: { // motor A
                        boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[8], 2), ec_);
                        for(int i=0;i<2;i++)
                        {
                            motor_current_buffer[i] = buffer_data[8+i];
                        }
                        state = 5;
                        break;
                    }
            case 5: { // steer deg
                        boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[10], 1), ec_);
                        for(int i=0;i<1;i++)
                        {
                            steer_buffer[i] = buffer_data[10+i];
                        }
                        state = 6;
                        break;
                    }
            case 6: { // gear
                        boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[11], 1), ec_);
                        for(int i=0;i<1;i++)
                        {
                            gear_buffer[i] = buffer_data[11+i];
                        }
                        state = 7;
                        break;
                    }
            case 7: { // yaw
                        boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[12], 2), ec_);
                        for(int i=0;i<2;i++)
                        {
                            yaw_buffer[i] = buffer_data[12+i];
                        }
                        state = 8;
                        break;
                    }
            case 8: { // end
                        boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[14], 1), ec_);
                        state = buffer_data[14] == END ? 0 : 9;
                        if (state == 9) {
                            ROS_DEBUG_STREAM("parse error 3 : ->" << (int) buffer_data[14]);
                            state = 0;
                        }
                        break;
                    }
            default: {
                         state = 0;
                         break;
                     }
        }
    } 
*/                     
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

    private_node.param<int>("baud_rate", baud_rate_, 115200);
    private_node.param<int>("control_rate", control_rate_, 10);
    private_node.param<int>("sensor_rate", sensor_rate_, 5);

    private_node.param<double>("wheelbase", wheelbase_, 1.85);
    private_node.param<double>("tread", tread_, 0.9);
    private_node.param<double>("wheel_diameter", wheel_diameter_, 0.5);
    private_node.param<double>("maxsteer", maxsteer_, 1);
    private_node.param<double>("steer_radius", steer_radius_, 3.5);
    private_node.param<double>("pid_rate", pid_rate_, 50.0);

    private_node.param<double>("maximum_encoding", maximum_encoding_, 32.0);
    private_node.param<bool>("publish_tf", publish_tf_, true);

    // 速度1m/s的情况下每个控制周期的脉冲数
    pulse_per_cycle_ = reduction_ratio_ * encoder_resolution_ / (M_PI * wheel_diameter_ * pid_rate_);

    if (init()) {
        odom_pub_ = node.advertise<nav_msgs::Odometry>("wheel_odom", 10);
        info_pub_ = node.advertise<zhongqi_driver::vehicle_info>("vehicle_info", 10);

        //        ros::Subscriber cmd_sub = node.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &ChassisDriver::twist_callback, this);
        ros::Subscriber ackermann_sub = node.subscribe<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 10, &ChassisDriver::ackermann_callback, this);
        ros::Timer send_speed_timer = node.createTimer(ros::Duration(1.0 / control_rate_), &ChassisDriver::send_speed_callback, this);

        ros::Timer pub_info_timer = node.createTimer(ros::Duration(1.0 / control_rate_), &ChassisDriver::pub_info_callback, this);

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

