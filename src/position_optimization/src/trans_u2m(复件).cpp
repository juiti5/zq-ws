#include <ros/ros.h>
//#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//#include <visualization_msgs/Marker.h>

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

using namespace std;
//using Eigen::MatrixXd;
//using Eigen::VectorXd;
bool position_calculation();

geometry_msgs::PoseWithCovarianceStamped amcl_msg_;
geometry_msgs::PointStamped pos_rear_msg_;
geometry_msgs::PointStamped pos_front_msg_;

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

inline void limit_deque(std::deque<Eigen::Vector2d> &deque_data)
{
    while (deque_data.size() > buffer_size_)
    {
        deque_data.pop_front();
    }
}

void amcl_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    amcl_msg_ = *msg.get();
    /*
    double _x = amcl_msg_.pose.pose.position.x;
    double _y = amcl_msg_.pose.pose.position.y;
    double _qx = amcl_msg_.pose.pose.orientation.x;
    double _qy = amcl_msg_.pose.pose.orientation.y;
    double _qz = amcl_msg_.pose.pose.orientation.z;
    double _qw = amcl_msg_.pose.pose.orientation.w;

    transform_.setOrigin(tf::Vector3(_x, _y, 0.0));
    transform_.setRotation(tf::Quaternion(_qx, _qy, _qz, _qw));
    */
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
        cout << "rotation_.determinant() > 0" << rotation_.determinant() << endl;
        trans_ = Eigen::Vector3d(p1.x(), p1.y(), 0) - rotation_ * Eigen::Vector3d(p2.x(), p2.y(), 0);
        //trans_ = Eigen::Vector3d(p2.x(), p2.y(), 0) - rotation_ * Eigen::Vector3d(p1.x(), p1.y(), 0);
    }
    else
    {
        //trans_ = org_trans_;
        cout << "rotation_.determinant()" << rotation_.determinant() << endl;
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

void position_front_callback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    pos_front_msg_ = *msg.get();
}
void position_callback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    pos_rear_msg_ = *msg.get();
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
                ROS_INFO_STREAM(count << "---" << msg->point.x << " " << msg->point.y << " "
                                      << local_transform.getOrigin().x() << " " << local_transform.getOrigin().y() << " "
                                      << trans_.x() << " " << trans_.y() << " " << tf::getYaw(map_to_odom_transform_.getRotation()));
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
    double ta = amcl_msg_.header.stamp.toSec();
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
    private_node.param<int>("rate", rate_, 20);

    ros::Subscriber amcl_sub = node.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, &amcl_callback);

    ros::Subscriber position_rear_sub = node.subscribe<geometry_msgs::PointStamped>("location_pos", 10, &position_callback);
    ros::Subscriber position_front_sub = node.subscribe<geometry_msgs::PointStamped>("location_pos_2", 10, &position_front_callback);
    ros::Timer pub_timer = node.createTimer(ros::Duration(0.1), &pub_callback);

    ros::spin();

    return 0;
}
