#ifndef NODE_H
#define NODE_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2/utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>

class Node{
public:
    Node(const int& id, const geometry_msgs::PoseStamped& from,const geometry_msgs::PoseStamped& to,const double radius,Node* const lastnode,Node* const nextnode);
    
    ~Node(){};
    
    void setID(int id){id_ = id;}
    int getID() const{return id_;}
    const geometry_msgs::PoseStamped& from() const {return from_;}
    const geometry_msgs::PoseStamped& to() const {return to_;}
    Node* last() const {return lastnode_;}
    Node* next() const {return nextnode_;}
    std::vector<geometry_msgs::PoseStamped>& path() {return path_;}
    const std::vector<geometry_msgs::PoseStamped>& reversePath() const {return reversepath_;}

    void setFrom(const geometry_msgs::PoseStamped& from){from_ = from;}
    void setTo(const geometry_msgs::PoseStamped& to){to_ = to;}
    void setRadius(const double& r){radius_=r;}
    void setLast(Node* last){lastnode_ = last;}
    void setNext(Node* next){nextnode_ = next;}
    
private:
    void setReversePath();
    int sign(double a){return a>0?1:-1;}
    int dequals(double a, double b) {return fabs(a-b) < 0.000001;}
    geometry_msgs::Point intersect(double a1,double b1,double c1,double a2,double b2,double c2);
    double normalizeTheta(double theta);
    int id_;
    geometry_msgs::PoseStamped from_, to_;
    double radius_;
    std::vector<geometry_msgs::PoseStamped> path_, reversepath_;
    Node* lastnode_;
    Node* nextnode_;
};

#endif //NODE_H
