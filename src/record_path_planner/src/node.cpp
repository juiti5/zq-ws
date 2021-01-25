#include <record_path_planner/node.h>

Node::Node(const int& id,const geometry_msgs::PoseStamped& from,const geometry_msgs::PoseStamped& to,const double radius,Node* const lastnode,Node* const nextnode):id_(id),from_(from), to_(to), radius_(radius), lastnode_(lastnode), nextnode_(nextnode){
//    ROS_INFO("path:ID-%d,R-%.2f",id_, radius_);
    
    double sx = from.pose.position.x;
    double sy = from.pose.position.y;
    double ex = to.pose.position.x;
    double ey = to.pose.position.y;
    double dx = ex - sx;
    double dy = ey - sy;
//    ROS_INFO("sx:%.2f,sy:%.2f,ex:%.2f,ey:%.2f,dx:%.2f,dy:%.2f", sx,sy,ex,ey,dx,dy);
    
    double dist = sqrt(dx*dx+dy*dy);
    
    geometry_msgs::PoseStamped point; 
    point.header.frame_id = "map";
//    point.header.stamp = from.header.stamp;
    if(radius == -1){
        if(dy == 0){
//            ROS_INFO("path_1");
            for(double i = sx; abs(abs(ex)-abs(i)) > 0.1; i += sign(dx)*0.05){
//                ROS_INFO("P1=x:%.2f, y:%.2f", i,sy);
                point.pose.position.x = i;
                point.pose.position.y = sy;
                point.pose.position.z = 0;
                point.pose.orientation = from.pose.orientation;
                path_.push_back(point);
            }
//            path_.push_back(to);
        }
        else if(dx == 0){
//            ROS_INFO("path_2");
            for(double i = sy; abs(abs(ey)-abs(i)) > 0.1; i += sign(dy)*0.05){
//                ROS_INFO("P2=x:%.2f, y:%.2f", sx,i);
                point.pose.position.x = sx;
                point.pose.position.y = i;
                point.pose.position.z = 0;
                point.pose.orientation = from.pose.orientation;
                path_.push_back(point);
            }
//            path_.push_back(to);
        }
        else{
//            ROS_INFO("path_3");
            double x,y,lx,ly;
            double k = dy/dx;
            double max = std::max(sx,ex);
            double min = std::min(sx,ex);  
            for(double i = 0; dist-i > 0.05; i += 0.1){
                x = sqrt((i*i)/(k*k+1))+sx;
                x = (x>sx && x>ex)?2*max-x:x;
                x = (x<sx && x<ex)?2*min-x:x;
                y=(x-sx)*k+sy;
                double ddx = x - lx;
                double ddy = y - ly;
                geometry_msgs::Quaternion quat;
                if(x == sx){
                    quat = from.pose.orientation;
                }else{
                    double th = atan2(ddy,ddx);
                    quat = tf::createQuaternionMsgFromYaw(th);
                }
//                ROS_INFO("P3=x:%.2f, y:%.2f", x,y);
                point.pose.position.x = x;
                point.pose.position.y = y;
                point.pose.position.z = 0;
                point.pose.orientation = quat;
                path_.push_back(point);
                lx = x;
                ly = y;
            }
//            path_.push_back(to);
        }
    }
    else{
        double a1,b1,c1,a2,b2,c2;
        
        double th1 = tf2::getYaw(from.pose.orientation);
        double th2 = tf2::getYaw(to.pose.orientation);
        double v1 = normalizeTheta(th1+M_PI_2);
        double v2 = normalizeTheta(th2+M_PI_2);        
        a1 = sin(th1);
        b1 = -cos(th1);
        c1 = -sin(th1)*sx+cos(th1)*sy;
        a2 = sin(v1);
        b2 = -cos(v1);
        c2 = -sin(v1)*ex+cos(v1)*ey;
        geometry_msgs::Point p1 = intersect(a1,b1,c1,a2,b2,c2); 
        
        double v3 = atan2(p1.y-ey,p1.x-ex);
//        ROS_INFO("P1=x:%.2f,y:%.2f,th:%.2f", p1.x,p1.y,v3);
        
        a1 = sin(th2);
        b1 = -cos(th2);
        c1 = -sin(th2)*ex+cos(th2)*ey;
        a2 = sin(v2);
        b2 = -cos(v2);
        c2 = -sin(v2)*sx+cos(v2)*sy;
        geometry_msgs::Point p2 = intersect(a1,b1,c1,a2,b2,c2);
        double v4 = atan2(p2.y-sy,p2.x-sx); 
//        ROS_INFO("P2=x:%.2f,y:%.2f,th:%.2f", p2.x,p2.y,v4);

        
        double vx1 = cos(v3)*radius;
        double vy1 = sin(v3)*radius;
        double vx2 = cos(v4)*radius;
        double vy2 = sin(v4)*radius;
//        ROS_INFO("th1:%.2f,th2:%.2f,v1:%.2f,v2:%.2f", th1,th2,v1,v2);
//        ROS_INFO("vx1:%.2f,vy1:%.2f,vx2:%.2f,vy2:%.2f", vx1,vy1,vx2,vy2);
        
        double d2 = dist*dist;
        double r2 = (radius*radius)*2;
//        
        if((d2>r2||dequals(d2,r2)) && dist>0){
            a1 = sin(th1);
            b1 = -cos(th1);
            c1 = (-sin(th1)*sx+cos(th1)*sy)+a1*vx1+b1*vy1;
            a2 = sin(th2);
            b2 = -cos(th2);
            c2 = (-sin(th2)*ex+cos(th2)*ey)+a2*vx2+b2*vy2;
//            ROS_INFO("a1:%.2f,b1:%.2f,c1:%.2f,a2:%.2f,b2:%.2f,c2:%.2f", a1,b1,c1,a2,b2,c2);
            geometry_msgs::Point cen = intersect(a1,b1,c1,a2,b2,c2);  
//            ROS_INFO("O=x:%.2f,y:%.2f", cen.x,cen.y);
            
            a1 = sin(th1);
            b1 = -cos(th1);
            c1 = -sin(th1)*sx+cos(th1)*sy;
            a2 = sin(v1);
            b2 = -cos(v1);
            c2 = -sin(v1)*cen.x+cos(v1)*cen.y;
            geometry_msgs::Point p3 = intersect(a1,b1,c1,a2,b2,c2);
//            ROS_INFO("P3=x:%.2f,y:%.2f", p3.x,p3.y);
            
            a1 = sin(th2);
            b1 = -cos(th2);
            c1 = -sin(th2)*ex+cos(th2)*ey;
            a2 = sin(v2);
            b2 = -cos(v2);
            c2 = -sin(v2)*cen.x+cos(v2)*cen.y;
            geometry_msgs::Point p4 = intersect(a1,b1,c1,a2,b2,c2);
//            ROS_INFO("P4=x:%.2f,y:%.2f", p4.x,p4.y);
            double ang_start = std::atan2(p3.y-cen.y, p3.x-cen.x);
            double ang_end = std::atan2(p4.y-cen.y, p4.x-cen.x);
            double c1,s1,c2,s2;
            c1 = cos(ang_start);
            s1 = sin(ang_start);
            c2 = cos(ang_end);
            s2 = sin(ang_end);
//            ROS_INFO("ang=s:%.2f,e:%.2f", ang_start,ang_end);
//            ROS_INFO("ang=c1:%.2f,s1:%.2f,c2:%.2f,s2:%.2f", c1,s1,c2,s2);
            //double dth = normalizeTheta(ang_end-ang_start);
//            double si = p4.x - p3.x;
            
//            for(double i = p3.x; (p4.x-i)>0.2;t += si*0.1){

 /*           for(double i = p3.x; abs(abs(p4.x)-abs(i)) > 0.2; i += sign(si)*0.1){
            
                double cy = sqrt(radius*radius-(i-p3.x)*(i-p3.x))+p3.y;
               
                ROS_INFO("R=x:%.2f,y:%.2f", i,cy);
            } 
            */
            double start = ang_start<0?ang_start+M_PI*2:ang_start;
            double end = ang_end<0?ang_end+M_PI*2:ang_end;
            double da = abs(ang_end - ang_start);
            da = da>M_PI?M_PI*2-da:da;
            
            double dth = ang_end - ang_start;
            int si = sign(dth);
            si = abs(dth)>M_PI?-1:si;
//            ROS_INFO("ang=s:%.2f,e:%.2f,da:%.2f,dth:%.2f,si:%d", ang_start,ang_end,da,dth,si);
            double lx,ly;
            for(double i = 0; abs(i)<da; i += si*0.1){
                double t = ang_start + i;
/*                if(abs(ang_start)>M_PI_2){
                    t = ang_start - i;
                }else{
                    t = ang_start + i;
                }*/
                 
//                ROS_INFO("ang=s:%.2f,e:%.2f", i,ang_end);
                double cx = cen.x + std::cos(t)*radius;
                double cy = cen.y + std::sin(t)*radius;
                
                geometry_msgs::Quaternion q;
                if(i == 0){
                    q = from.pose.orientation;
                }else{
                    double dx = cx - lx;
                    double dy = cy - ly;
                    double th = atan2(dy,dx);
                    
                    q = tf::createQuaternionMsgFromYaw(th);
                }
                lx = cx;
                ly = cy;
                
                point.pose.position.x = cx;
                point.pose.position.y = cy;
                point.pose.position.z = 0;
                point.pose.orientation = q;
                path_.push_back(point);
               
//                ROS_INFO("R=x:%.2f,y:%.2f", cx,cy);
            }
//            path_.push_back(to);
            
                      
        }else{
            ROS_ERROR("can not calculat......");
            ROS_INFO("dist:%.2f,r2:%.2f,equal:%d,vy2:%.2f", d2,r2,dequals(d2,r2),vy2);
        }
    }
/*
    for (size_t i = 0; i < path_.size(); i++) {
        ROS_INFO("S=x:%.2f,y:%.2f", path_[i].pose.position.x,path_[i].pose.position.y);

    }
    
*/
}

geometry_msgs::Point Node::intersect(double a1,double b1,double c1,double a2,double b2,double c2){
    geometry_msgs::Point p;
    double D=a1*b2-a2*b1;
//    ROS_INFO("a1:%.2f,a2:%.2f,b1:%.2f,b2:%.2f,D:%.2f", a1,a2,b1,b2,D);
    if(abs(D)<0.00001){
        ROS_ERROR("no intersect......");
    }
    else{
        p.x=(b1*c2-b2*c1)/D;
        p.y=(a2*c1-a1*c2)/D;
        //        ROS_INFO("intersect=x:%.2f,y:%.2f", p.x,p.y);  
    }
    return p;
}

double Node::normalizeTheta(double theta){
    if (theta >= -M_PI && theta < M_PI)
        return theta;
    double multiplier = std::floor(theta / (2*M_PI));
    theta = theta - multiplier*2*M_PI;
    if (theta >= M_PI)
        theta -= 2*M_PI;
    if (theta < -M_PI)
        theta += 2*M_PI;
    return theta;
}

