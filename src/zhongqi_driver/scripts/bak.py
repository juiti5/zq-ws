#!/usr/bin/env python

import rospy
import roslaunch
import os
#from server_control.msg import Single
from geometry_msgs.msg import Twist
import threading


devicepath = "/dev"
slaming = False
naving = False

class device:
    def __init__(self, path):
        self.car = False
        self.imu = False
        self.camera = False
        self.front_tag = False
        self.rear_tag = False
        self.front_lidar = False
        self.rear_lidar = False
        self.path = path
        self.devicelist = []
        self.checkDevice()
        
    def checkDevice(self):
        os.chdir(self.path)
        self.devicelist = []
        for i in os.listdir(os.getcwd()):
            if (
                i == "ucar"
                or "uimu"
                or "ucamera"
                or "ufront"
                or "urear"
                or "ulidar_front"
                or "ulidar_rear"
            ):
                self.devicelist.append(i)
        if "ucar" in self.devicelist:
            self.car = True
        else:
            self.car = False
        if "uimu" in self.devicelist:
            self.imu = True
        else:
            self.imu = False
        if "ucamera" in self.devicelist:
            self.camera = True
        else:
            self.camera = False
        if "ufront" in self.devicelist:
            self.front_tag = True
        else:
            self.front_tag = False
        if "urear" in self.devicelist:
            self.rear_tag = True
        else:
            self.rear_tag = False
        if "ulidar_front" in self.devicelist:
            self.front_lidar = True
        else:
            self.front_lidar = False
        if "ulidar_rear" in self.devicelist:
            self.rear_lidar = True
        else:
            self.rear_lidar = False

 
def init():
    print("check!")
    chdev = device(devicepath)
    print('car:%d\nimu:%d\ncamera:%d\nfront_tag:%d\nrear_tag:%d\nfront_lidar:%d\nrear_lidar:%d' %(chdev.car,chdev.imu,chdev.camera,chdev.front_tag,chdev.rear_tag,chdev.front_lidar,chdev.rear_lidar))    
#    os.system('roslaunch zhongqi_driver driver.launch') 
#    os.system('roslaunch zhongqi_driver ws.launch') 
    print("init_ok!!!")

def start_slam():
    global slaming
    slaming = True
    os.system('roslaunch lidar_slam zhongqi_slam.launch')
    print("start_slam!")
    

def stop_slam():
    global slaming
    os.system('/home/autolabor/save_map tttt')
    print("kill_slam!")
#    rospy.sleep(10)
    os.system('rosnode kill /cartographer_node')
    os.system('rosnode kill /cartographer_occupancy_grid_node')
    os.system('rosnode kill /rviz')
    slaming = False
    print("stop_slam!")
    

def start_navi():
    global naving
    naving = True
    os.system('rosrun zhongqi_navigation start.py')
#    os.system('rosrun zhongqi_driver pure_pursuit.py')

    print("start_navi!")


def stop_navi():
    global naving
    os.system('rosnode kill /amcl')
    os.system('rosnode kill /map_server')
    os.system('rosnode kill /move_base')
    os.system('rosnode kill /rviz')
    os.system('rosnode kill /u2m_node')
    naving = False        
    print("stop_navi!")


def single_callback(single):
    global slaming
    global naving
    global pid
    if slaming:
        print("slaming!")
    if naving:
        print("naving!") 

    if single.angular.y == 3 and not slaming and not naving:
        start_slam()
    if single.angular.y == 4 and not naving:
        stop_slam()
    if single.angular.y == 1 and not slaming and not naving:
        start_navi()
    if single.angular.y == 0 and not slaming:
        stop_navi()
        
    '''
    if single.slam_on and not running:
        start_slam()
    if single.slam_off:
        stop_slam()
    if single.navi_on and not running:
        start_navi()
    if single.navi_off:
        stop_navi()
    '''

if __name__ == "__main__":
    rospy.init_node("server_control_node")
    init()
#    rospy.Subscriber("/control_single", Single, single_callback, queue_size=1)
    rospy.Subscriber("/car_goal_run", Twist, single_callback)
    
    rospy.spin()
