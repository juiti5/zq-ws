#!/usr/bin/env python

import rospy
import roslaunch
import os
import time
#from server_control.msg import Single
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from multiprocessing.pool import ThreadPool
import threading
import thread

devicepath = "/dev"
slaming = False
naving = False
first = True
class device:
    def __init__(self, path):
        self.car = False
        self.imu = False
        self.front_camera = False
        self.rear_camera = False
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
                or "ucamera_front"
                or "ucamera_rear"
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
        if "ucamera_front" in self.devicelist:
            self.front_camera = True
        else:
            self.camera = False
        if "ucamera_rear" in self.devicelist:
            self.rear_camera = True
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


def topic_switch(switch):
    switch_msg = Bool()
    if switch:
      switch_msg.data = True    
    else:
      switch_msg.data = False
    switch_pub.publish(switch_msg)
 
 
def init():
    print("check!")
    chdev = device(devicepath)
    print('car:%d\nimu:%d\nfront_camera:%d\nrear_camera:%d\nfront_tag:%d\nrear_tag:%d\nfront_lidar:%d\nrear_lidar:%d' %(chdev.car,chdev.imu,chdev.front_camera,chdev.rear_camera,chdev.front_tag,chdev.rear_tag,chdev.front_lidar,chdev.rear_lidar))    
#    i=0
#    while i<500:
#      topic_switch(True)
#      i+=1
    print("init_ok!!!")

def start_slam():
    global slaming
    slaming = True
    print("start_slam!")
#    os.system('roslaunch lidar_slam zhongqi_slam.launch')
    thread.start_new_thread(os.system, ('roslaunch lidar_slam zhongqi_slam.launch',))   

def stop_slam():
    global slaming
    os.system('/home/autolabor/zhongqi_ws/scripts/save_map bbbb')
    os.system('rosnode kill /cartographer_node')
    os.system('rosnode kill /cartographer_occupancy_grid_node')
    os.system('rosnode kill /rviz')
    slaming = False
    print("stop_slam!")
    

def start_navi():
    global naving
    naving = True
    print("start_navi!")
#    os.system('rosrun zhongqi_navigation start.py')
    thread.start_new_thread(os.system, ('rosrun zhongqi_navigation start.py',))   
#    pool = ThreadPool(processes=1)
#    pool.apply_async(os.system, ('rosrun zhongqi_navigation start.py',))
def stop_navi():
    global naving
    os.system('rosnode kill /amcl')
    os.system('rosnode kill /map_server')
    os.system('rosnode kill /move_base')
    os.system('rosnode kill /rviz')
    os.system('rosnode kill /u2m_node')
    os.system('rosnode kill /pure_pursuit')
    naving = False        
    print("stop_navi!")


def single_callback(single):
    global slaming
    global naving
    global pid
    global first

    print("single_callback!")
    if slaming:
        print("slaming!")
    if naving:
        print("naving!") 
    if single.angular.y == 3213 and not slaming and not naving:
        first = False
        topic_switch(True)
        start_slam()
    if single.angular.y == 3214 and not naving:
        first = False
        topic_switch(True)
        stop_slam()
    if single.angular.y == 3211 and not slaming and not naving:
        first = False
        topic_switch(False)
        start_navi()
    if single.angular.y == 3210 and not slaming:
        first = False
        topic_switch(True)
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

def test():
  global first
  '''
  import Tkinter
 
  root_window = Tkinter.Tk()
  root_window.title('Tkinter_Demo')
  root_window.geometry('400x300')
 
  m_text = Tkinter.Text(root_window)
  m_text.insert(Tkinter.CURRENT, 'hello \n')
  m_text.insert(Tkinter.END, 'world \n')
  m_text.insert(Tkinter.END, 'nono')
  m_text.pack()
  root_window.mainloop()
  '''
  while first:
    topic_switch(True) 
#    print("topic_switch!")

if __name__ == "__main__":
    rospy.init_node("server_control_node")
#    rospy.Subscriber("/control_single", Single, single_callback, queue_size=1)
    switch_pub = rospy.Publisher('/topic_switch', Bool, queue_size=1) 
    rospy.Subscriber("/car_goal_run", Twist, single_callback)    
    init()

    t1 = threading.Thread(target=test)
    t1.start()   
    rospy.spin()
