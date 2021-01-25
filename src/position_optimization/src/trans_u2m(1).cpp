#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

//#include <eigen3/Eigen/Core>
//#include <Eigen/Dense>
//#include <eigen3/Eigen/SVD>
#include <tf_conversions/tf_eigen.h>

//#include <vector>
//#include <deque>
//#include <iostream>

//#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <position_optimization/pose_init.h>

using namespace std;
//using Eigen::MatrixXd;
//using Eigen::VectorXd;

#define PI 3.14159
bool position_calculation();
Eigen::Matrix3d transform(double ox, double oy, double oyaw, double tx, double ty, double ro);
ros::Publisher goal_pub_;
ros::Publisher init_pub_;
ros::Publisher u2apos_pub_;
ros::Publisher pose_pub_;
ros::Publisher local_path_pub_;
ros::Publisher global_path_pub_;
geometry_msgs::PoseStamped goal_msg_;
geometry_msgs::PoseStamped map_goal_;

bool pub_ = false;
double trans_x_;
double trans_y_;
double trans_yaw_;
double position_yaw_;
double m2px;
double m2py;
Eigen::Matrix3d origin_r_;
Eigen::Matrix3d origin_init_;
Eigen::Matrix3d origin_zero_;
Eigen::Matrix3d rotation_z_;
Eigen::Matrix3d rotation_x_;
Eigen::Matrix3d transfrom_;
Eigen::Matrix3d result_r_;
Eigen::Matrix3d result_init_;
Eigen::Matrix3d result_zero_;

geometry_msgs::PoseWithCovarianceStamped initpose;
geometry_msgs::PoseWithCovarianceStamped amcl_msg_;
nav_msgs::Odometry kfu_msg_;
nav_msgs::Odometry luo_msg_;
nav_msgs::Odometry pf_msg_;
geometry_msgs::PoseWithCovarianceStamped upos;
geometry_msgs::PointStamped pos_rear_msg_;
geometry_msgs::PointStamped pos_front_msg_;
nav_msgs::Path mlocal_path, mglobal_path, ulocal_path, uglobal_path;

position_optimization::pose_init pose_init_msg_;

std::string odom_frame_, base_frame_, map_frame_;
bool start_flag_ = true;
bool real_pos_;
int rate_;
double distance_interval_;
int buffer_size_;

tf::Transform map_to_odom_transform_;
tf::Transform transform_;

std::deque<Eigen::Vector2d> map_points_;
std::deque<Eigen::Vector2d> odom_points_;

Eigen::Matrix3d rotation_;
Eigen::Vector3d trans_;
Eigen::Vector3d org_trans_;
/*

/move_base/GlobalPlanner/plan

/move_base/TebLocalPlannerROS/local_plan
*/
inline void limit_deque(std::deque<Eigen::Vector2d> &deque_data)
{
    while (deque_data.size() > buffer_size_)
    {
        deque_data.pop_front();
    }
}

void mlocal_path_callback(const nav_msgs::Path::ConstPtr &msg)
{
  mlocal_path = *msg.get();
  
  geometry_msgs::PoseStamped temp_pose;
  temp_pose.header = mglobal_path.header;
  for (auto i : mlocal_path.poses)
  {  
    double _x = i.pose.position.x;
    double _y = i.pose.position.y;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(i.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    Eigen::Matrix3d ul_path = transform(_x, _y, yaw, m2px, m2py, trans_yaw_);
    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(ul_path(2,0));
    temp_pose.pose.position.x = ul_path(0,0);    
    temp_pose.pose.position.y = ul_path(1,0);
    temp_pose.pose.orientation = q;
    ulocal_path.poses.push_back(temp_pose);
  }
  ulocal_path.header.frame_id = "map";
  ulocal_path.header.stamp = ros::Time::now();
//  ulocal_path.poses.pose.orientation = q;
//  ulocal_path.poses.pose.position.x = ul_path(0,0);
//  ulocal_path.poses.pose.position.y = ul_path(1,0);

  local_path_pub_.publish(ulocal_path);
}

void mglobal_path_callback(const nav_msgs::Path::ConstPtr &msg)
{
  mglobal_path = *msg.get();
  geometry_msgs::PoseStamped temp_pose;
  temp_pose.header = mglobal_path.header;
  for (auto i : mglobal_path.poses)
  {
    double _x = i.pose.position.x;
    double _y = i.pose.position.y;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(i.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    Eigen::Matrix3d ug_path = transform(_x, _y, yaw, m2px, m2py, trans_yaw_);
    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(ug_path(2,0));
    
    temp_pose.pose.position.x = ug_path(0,0);    
    temp_pose.pose.position.y = ug_path(1,0);
    temp_pose.pose.orientation = q;
    uglobal_path.poses.push_back(temp_pose);
  }
    
  uglobal_path.header.frame_id = "map";
  uglobal_path.header.stamp = ros::Time::now();

  global_path_pub_.publish(uglobal_path);
}

void luo_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    luo_msg_ = *msg.get();
    double _x = luo_msg_.pose.pose.position.x;
    double _y = luo_msg_.pose.pose.position.y;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(luo_msg_.pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    Eigen::Matrix3d u_pos = transform(_x, _y, yaw, m2px, m2py, trans_yaw_);
 
    
    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(u_pos(2,0));
  
    pf_msg_.header.frame_id = "map";
    pf_msg_.child_frame_id = base_frame_;
    pf_msg_.header.stamp = ros::Time::now();
  
    pf_msg_.pose.pose.orientation = q;
    pf_msg_.pose.pose.position.x = u_pos(0,0);
    pf_msg_.pose.pose.position.y = u_pos(1,0);
    pf_msg_.pose.pose.position.z = u_pos(2,0);
  
    pf_msg_.twist.twist.linear.x =0;
    pf_msg_.twist.twist.linear.y =0;
    pf_msg_.twist.twist.angular.z =0;
  
    pose_pub_.publish(pf_msg_);
}

void kfu_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    kfu_msg_ = *msg.get();
    double _x = kfu_msg_.pose.pose.position.x;
    double _y = kfu_msg_.pose.pose.position.y;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(kfu_msg_.pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
//    Eigen::Matrix3d u_pos = transform(_x, _y, yaw, m2px, m2py, trans_yaw_);
    Eigen::Matrix3d u_pos = transform(_x, _y, yaw, trans_x_, trans_y_, -trans_yaw_);
 
    geometry_msgs::Quaternion upos_quat = tf::createQuaternionMsgFromYaw(u_pos(2,0));
    upos.header.stamp = ros::Time::now();
    //upos.header.frame_id = "map";
    upos.pose.pose.position.x = u_pos(0,0);
    upos.pose.pose.position.y = u_pos(1,0);
    upos.pose.pose.position.z = 0;
    upos.pose.pose.orientation = upos_quat;
    u2apos_pub_.publish(upos);
    
    /*
    double _qx = amcl_msg_.pose.pose.orientation.x;
    double _qy = amcl_msg_.pose.pose.orientation.y;
    double _qz = amcl_msg_.pose.pose.orientation.z;
    double _qw = amcl_msg_.pose.pose.orientation.w;

    transform_.setOrigin(tf::Vector3(_x, _y, 0.0));
    transform_.setRotation(tf::Quaternion(_qx, _qy, _qz, _qw));
    */
}

void amcl_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  amcl_msg_ = *msg.get();
}

/*
void pub_callback(const ros::TimerEvent &)
{
    ROS_INFO("amcl stamp value is: %f", amcl_msg_.header.stamp.toSec());
    ROS_INFO("pose stamp value is: %f", pos_rear_msg_.header.stamp.toSec());
}
*/

void point_match()
{
    // center of mass
    Eigen::Vector2d p1(0, 0), p2(0, 0);
    size_t point_size = map_points_.size();
    for (size_t i = 0; i < point_size; i++)
    {
        p1 += map_points_[i];
        p2 += odom_points_[i];
    }
    p1 = Eigen::Vector2d(p1 / point_size);
    p2 = Eigen::Vector2d(p2 / point_size);

    // remove the center
    std::vector<Eigen::Vector2d> q1(point_size), q2(point_size);
    for (size_t i = 0; i < point_size; i++)
    {
        q1[i] = map_points_[i] - p1;
        q2[i] = odom_points_[i] - p2;
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < point_size; i++)
    {
        W += Eigen::Vector3d(q1[i].x(), q1[i].y(), 0) * Eigen::Vector3d(q2[i].x(), q2[i].y(), 0).transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    rotation_ = U * (V.transpose());
    //rotation_ = V * (U.transpose());
    //cout << "rotation_" << rotation_ << endl;
    if (rotation_.determinant() > 0)
    {
        //cout << "rotation_.determinant() > 0" << rotation_.determinant() << endl;
        trans_ = Eigen::Vector3d(p1.x(), p1.y(), 0) - rotation_ * Eigen::Vector3d(p2.x(), p2.y(), 0);
        //trans_ = Eigen::Vector3d(p2.x(), p2.y(), 0) - rotation_ * Eigen::Vector3d(p1.x(), p1.y(), 0);
    }
    else
    {
        //trans_ = org_trans_;
        //cout << "rotation_.determinant()" << rotation_.determinant() << endl;
        trans_ = Eigen::Vector3d(p1.x(), p1.y(), 0) - rotation_ * Eigen::Vector3d(p2.x(), p2.y(), 0);
        //trans_ = Eigen::Vector3d(p2.x(), p2.y(), 0) - rotation_ * Eigen::Vector3d(p1.x(), p1.y(), 0);

        Eigen::Vector3d diff = Eigen::Vector3d(p1.x() - org_trans_.x(), p1.y() - org_trans_.y(), 0);
        // p2->1  diff->2
        double dot = p2.x() * diff.x() + p2.y() * diff.y();
        double det = p2.x() * diff.y() - p2.y() * diff.x();
        double angle = atan2(det, dot);
        double c = cos(angle);
        double s = sin(angle);
        rotation_ << c, -s, 0,
            s, c, 0,
            0, 0, 1;
    }

    tf::Vector3 map_to_odom_origin;
    tf::vectorEigenToTF(trans_, map_to_odom_origin);
    map_to_odom_transform_.setOrigin(map_to_odom_origin);

    tf::Matrix3x3 map_to_odom_rot;
    tf::matrixEigenToTF(rotation_, map_to_odom_rot);
    map_to_odom_transform_.setBasis(map_to_odom_rot);

    // std::cout << "trans : " << trans_.x() << " " << trans_.y() << " " << std::endl;
    // std::cout << "rot : " << tf::getYaw(map_to_odom_transform_.getRotation()) << " " << std::endl
    //           << std::endl;
}

void init_pose_callback(const position_optimization::pose_init::ConstPtr &msg)
{
    pose_init_msg_ = *msg.get();
}

void position_front_callback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    pos_front_msg_ = *msg.get();
}
void position_callback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    pos_rear_msg_ = *msg.get();
    pub_ = true;
    bool real_pos = real_pos_;
    static int count;
    static tf::TransformListener tf_;
    //std::cout << "uwb" << start_flag_ << std::endl;
    //std::cout << "uwb" << count << std::endl;
    if (tf_.canTransform(odom_frame_, base_frame_, ros::Time()) && real_pos && count < 10)
    {
        tf::StampedTransform local_transform;
        tf_.lookupTransform(odom_frame_, base_frame_, ros::Time(), local_transform);

        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped.header.stamp = ros::Time::now();
        transform_stamped.header.frame_id = map_frame_;
        transform_stamped.child_frame_id = odom_frame_;

        if (start_flag_)
        {
            map_points_.emplace_back(msg->point.x, msg->point.y);
            odom_points_.emplace_back(local_transform.getOrigin().x(), local_transform.getOrigin().y());
            org_trans_ << msg->point.x - local_transform.getOrigin().x(), msg->point.y - local_transform.getOrigin().y(), 0;
            start_flag_ = false;
        }
        else
        {
            Eigen::Vector2d last_odom = odom_points_.back();
            double delta_x = local_transform.getOrigin().x() - last_odom.x();
            double delta_y = local_transform.getOrigin().y() - last_odom.y();
            if ((delta_x * delta_x + delta_y * delta_y) > (distance_interval_ * distance_interval_))
            {
                count++;
                map_points_.emplace_back(msg->point.x, msg->point.y);
                odom_points_.emplace_back(local_transform.getOrigin().x(), local_transform.getOrigin().y());
                limit_deque(map_points_);
                limit_deque(odom_points_);
                point_match();
                /*
                ROS_INFO_STREAM(count << "---" << msg->point.x << " " << msg->point.y << " "
                                      << local_transform.getOrigin().x() << " " << local_transform.getOrigin().y() << " "
                                      << trans_.x() << " " << trans_.y() << " " << tf::getYaw(map_to_odom_transform_.getRotation()));
               */
            }
        }
    }
    //std::cout << "trans : " << trans_.x() << " " << trans_.y() << " " << std::endl;
    //std::cout << "rot : " << tf::getYaw(map_to_odom_transform_.getRotation()) << " " << std::endl
    //          << std::endl;
}

//bool position_calculation()
void pub_callback(const ros::TimerEvent &)
{
    double diff_x = pos_front_msg_.point.x - pos_rear_msg_.point.x;
    double diff_y = pos_front_msg_.point.y - pos_rear_msg_.point.y;
    double x = pos_rear_msg_.point.x;
    double y = pos_rear_msg_.point.y;

    double distance = sqrt(pow(diff_x, 2) + pow(diff_y, 2));
    position_yaw_ = atan2(diff_y, diff_x);

    double ta = 0; //amcl_msg_.header.stamp.toSec();
    double tp = pos_rear_msg_.header.stamp.toSec();
    double dt = abs(ta - tp);
    //ROS_INFO("ERROR:%.2f,%.2f", ta, tp);
    if (distance > 0.70 || distance < 0.60 || distance == 0 || dt > 0.2)
    {
        //real_position_ = false;
        //check_area_ = false;
        //ROS_INFO_STREAM("ERROR" << real_position_);
        //ROS_INFO("ERROR:%.2f,%.2f", distance, dt);
        real_pos_ = false;
    }
    else
    {
        //   ROS_INFO("OK%.2f", distance);
        real_pos_ = true;
    }
}

void goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    goal_msg_ = *msg.get();
    pub_ = true;
}

Eigen::Matrix3d transform(double ox, double oy, double oyaw, double tx, double ty, double ro)
{
  Eigen::Vector3d vu(cos(oyaw), sin(oyaw), 0);
  Eigen::Vector3d m(0, 0, 0);
  Eigen::Vector3d ea(PI, 0, ro);
  Eigen::Quaterniond q;
  Eigen::Matrix3d origin;
  Eigen::Matrix3d rotation_z;
  Eigen::Matrix3d rotation_x;
  Eigen::Matrix3d transfrom;
  Eigen::Matrix3d result;
  origin << ox, 0, 0, oy, 1, 0, 0, 0, 1;
  rotation_z << cos(ro), -sin(ro), 0, sin(ro), cos(ro), 0, 0, 0, 1;
  rotation_x << 1, 0, 0, 0, cos(PI), -sin(PI), 0, sin(PI), cos(PI);
  transfrom << tx, 0, 0, ty, 1, 0, 0, 0, 1;
  result = rotation_x * rotation_z * (origin - transfrom);
  q = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitZ());     
  m = q * vu;
  result(2,0) = atan2(m[1], m[0]);
    
  return result;
}

void goal_pub_callback(const ros::TimerEvent &)
{
    if (pub_)
    {
        //double ox = goal_msg_.pose.position.x;
        //double oy = goal_msg_.pose.position.y;
        double rx = pos_rear_msg_.point.x;
        double ry = pos_rear_msg_.point.y;

        double init_x = pose_init_msg_.x;
        double init_y = pose_init_msg_.y;
        double init_yaw = pose_init_msg_.yaw;

        double tx = trans_x_;
        double ty = trans_y_;
        double ro = -trans_yaw_;
        
        double vx = pos_front_msg_.point.x - pos_rear_msg_.point.x;
        double vy = pos_front_msg_.point.y - pos_rear_msg_.point.y;
        double yaw_u = atan2(vy, vx);
/*
        origin_r_ << rx, 0, 0, ry, 1, 0, 0, 0, 1;
        origin_init_ << init_x, 0, 0, init_y, 1, 0, 0, 0, 1;
        rotation_z_ << cos(ro), -sin(ro), 0, sin(ro), cos(ro), 0, 0, 0, 1;
        rotation_x_ << 1, 0, 0, 0, cos(PI), -sin(PI), 0, sin(PI), cos(PI);
        transfrom_ << tx, 0, 0, ty, 1, 0, 0, 0, 1;

        result_r_ = rotation_x_ * rotation_z_ * (origin_r_ - transfrom_);
        result_init_ = rotation_x_ * rotation_z_ * (origin_init_ - transfrom_);
*/
        result_r_ = transform(rx, ry, yaw_u, tx, ty, ro);
        result_init_ = transform(init_x, init_y, init_yaw, tx, ty, ro);
        result_zero_ = transform(0, 0, 0, tx, ty, ro);
        
        m2px = result_zero_(0,0);
        m2py = result_zero_(1,0);

        /*
        cout << "origin_r_ \n " << origin_r_ << endl;
        cout << "rotation_x_ \n " << rotation_x_ << endl;
        cout << "rotation_z_ \n " << rotation_z_ << endl;
        cout << "transfrom_\n " << transfrom_ << endl;
        cout << "result_r_ \n " << result_r_ << endl;
        */
        //double vx = origin_f_(0, 0) - origin_r_(0, 0);
        //double vy = origin_f_(1, 0) - origin_r_(1, 0);
        
        /*
        vx = cos(yaw_u);
        vy = sin(yaw_u);
        double v_ix = cos(init_yaw);
        double v_iy = sin(init_yaw);
        */
        
        double mx = result_r_(0, 0);
        double my = result_r_(1, 0);

        double initx = result_init_(0, 0);
        double inity = result_init_(1, 0);
        /*
        Eigen::Vector3d vu(vx, vy, 0);
        Eigen::Vector3d init_vy(v_ix, v_iy, 0);
        Eigen::Vector3d m(0, 0, 0);
        Eigen::Vector3d init_m(0, 0, 0);

        Eigen::Vector3d ea(PI, 0, -trans_yaw_); //-trans_yaw_

        Eigen::Quaterniond q;
        q = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitZ());
        */
        /*
          Eigen::Quaterniond qr;
          qr = qmo_u * q;
          Eigen::Vector3d eulerAngle = qr.matrix().eulerAngles(0, 1, 2);
          double yaw_ = eulerAngle(2);
          cout << "YU " << eulerAngle << endl;
        */
        /*
        m = q * vu;
        init_m = q * init_vy;
        double yaw_ = atan2(m[1], m[0]);
        double init_yaw_m = atan2(init_m[1], init_m[0]);
        */
        double yaw_ = result_r_(2,0);
        double init_yaw_m = result_init_(2,0);
        //ROS_INFO("YAW:%.2f,%.2f,%.2f", position_yaw_, trans_yaw_, yaw_);

        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(yaw_);
        map_goal_.header.stamp = ros::Time::now();
        map_goal_.header.frame_id = "map";
        map_goal_.pose.position.x = mx;
        map_goal_.pose.position.y = my;
        map_goal_.pose.position.z = 0;
        map_goal_.pose.orientation = quat;
        goal_pub_.publish(map_goal_);

        static int count;
        if (count < 3)
        {
            count++;
            geometry_msgs::Quaternion init_quat = tf::createQuaternionMsgFromYaw(init_yaw_m);
            initpose.header.stamp = ros::Time::now();
            initpose.header.frame_id = "map";
            initpose.pose.pose.position.x = initx;
            initpose.pose.pose.position.y = inity;
            initpose.pose.pose.position.z = 0;
            initpose.pose.pose.orientation = init_quat;
            init_pub_.publish(initpose);
        }

        pub_ = false;
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "trans_u2m_node");
    ros::NodeHandle node;

    ros::NodeHandle private_node("~");
    //std::cout << "i"  << "n" << "i" << "t" << std::endl;
    private_node.param<std::string>("odom_frame", odom_frame_, std::string("odom"));
    private_node.param<std::string>("base_frame", base_frame_, std::string("car"));
    private_node.param<std::string>("map_frame", map_frame_, std::string("map"));
    private_node.param<int>("buffer_size", buffer_size_, 50);
    private_node.param<double>("distance_interval", distance_interval_, 0.1);
    private_node.param<double>("trans_x", trans_x_, 0.1);
    private_node.param<double>("trans_y", trans_y_, 0.1);
    private_node.param<double>("trans_yaw", trans_yaw_, 0.1);
    private_node.param<int>("rate", rate_, 20);
    
    ros::Subscriber mlocal_path_sub = node.subscribe<nav_msgs::Path>("/move_base/TebLocalPlannerROS/local_plan", 10, &mlocal_path_callback);
    ros::Subscriber mglobal_path_sub = node.subscribe<nav_msgs::Path>("/move_base/GlobalPlanner/plan", 10, &mglobal_path_callback);
    ros::Subscriber kfu_sub = node.subscribe<nav_msgs::Odometry>("/ku_pos", 10, &kfu_callback);
    ros::Subscriber luo_sub = node.subscribe<nav_msgs::Odometry>("/luo_pos", 10, &luo_callback);
    ros::Subscriber goal_sub = node.subscribe<geometry_msgs::PoseStamped>("/car_goal", 10, &goal_callback);
    ros::Subscriber init_pose_sub = node.subscribe<position_optimization::pose_init>("/init_pose", 10, &init_pose_callback);
    
    local_path_pub_ = node.advertise<nav_msgs::Path>("/local_path", 10);
    global_path_pub_ = node.advertise<nav_msgs::Path>("/global_path", 10);
    pose_pub_ = node.advertise<nav_msgs::Odometry>("/carpos", 10);
    goal_pub_ = node.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal_demo", 10);
    init_pub_ = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
    u2apos_pub_ = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("/u2apose", 10);
    ros::Timer goal_pub_timer = node.createTimer(ros::Duration(0.1), &goal_pub_callback);
    ros::Subscriber amcl_sub = node.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, &amcl_callback);
    ros::Subscriber position_rear_sub = node.subscribe<geometry_msgs::PointStamped>("location_pos", 10, &position_callback);
    ros::Subscriber position_front_sub = node.subscribe<geometry_msgs::PointStamped>("location_pos_2", 10, &position_front_callback);
    ros::Timer pub_timer = node.createTimer(ros::Duration(0.1), &pub_callback);

    ros::spin();

    return 0;
}
