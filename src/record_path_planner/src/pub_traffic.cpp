#include <ros/ros.h>
#include <control_detection/Light.h>

class Traffic {
public:
    Traffic();

    ~Traffic();
    
    void traffic_callback(const ros::TimerEvent &);   
    void run();

private:
    ros::NodeHandle nh_;

    ros::Timer pub_timer;
    ros::Publisher traffic_pub_;
    
    double now_, last_, dt_;
    bool first_;
    int state_;
};


Traffic::Traffic(): now_(0),last_(0),first_(true),state_(1){

}


Traffic::~Traffic() {
}


void Traffic::traffic_callback(const ros::TimerEvent &){
    control_detection::Light tl_msg;
    now_ = ros::Time::now().toSec();
    if(first_){
        last_ = now_;
        first_ = false;
    }
    dt_ = now_ - last_;
//    ROS_INFO("Traffic.(%.2f)",dt_);
    tl_msg.header.stamp = ros::Time::now();
    tl_msg.light_id = 0;
    
    switch(state_){
        case 1:
            ROS_INFO("GREEN.(%.2f)",dt_);
            tl_msg.light_color = 1;
            if(dt_>10){
                state_ = 2;
                last_ = now_;
            }
            break;
        case 2:
            ROS_INFO("YELLOW.(%.2f)",dt_);
            tl_msg.light_color = 0;
            if(dt_>5){
                state_ = 3;
                last_ = now_;
            }
        
            break;
        case 3:
            ROS_INFO("RED.(%.2f)",dt_);
            tl_msg.light_color = 0;
            if(dt_>10){
                state_ = 1;
                last_ = now_;
            }
        
            break;
    }
   
    traffic_pub_.publish(tl_msg);
}


void Traffic::run(){
    traffic_pub_ = nh_.advertise<control_detection::Light>("/traffic_light", 1, true);
    pub_timer = nh_.createTimer(ros::Duration(0.5), &Traffic::traffic_callback, this);
    
    ros::spin();
}


int main(int argc, char** argv){
    ros::init(argc, argv, "triffic_node");
    
    Traffic traffic;
    traffic.run();
}
