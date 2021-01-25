#!/usr/bin/env python
import rospy
import Tkinter
from geometry_msgs.msg import Twist
import threading
from multiprocessing.pool import ThreadPool

work=0
run=0
loop=0

def slamOnCallBack():
  global run
  global work
  switch_msg = Twist()
  work = 3213
  switch_msg.angular.x = run
  switch_msg.angular.y = work
#  print 'slamOnCallBack'
  switch_pub.publish(switch_msg)

def slamOffCallBack():
  global run
  global work
  switch_msg = Twist()
  work = 3214
  switch_msg.angular.x = run
  switch_msg.angular.y = work
#  print 'slamOffCallBack'
  switch_pub.publish(switch_msg)

def naviOnCallBack():
  global run
  global work
  switch_msg = Twist()
  work = 3211
  switch_msg.angular.x = run
  switch_msg.angular.y = work
#  print 'naviOnCallBack'
  switch_pub.publish(switch_msg)

def naviOffCallBack():
  global run
  global work
  switch_msg = Twist()
  work = 3210
  switch_msg.angular.x = run
  switch_msg.angular.y = work
#  print 'naviOffCallBack'
  switch_pub.publish(switch_msg)

def runCallBack():
  global run
  global work
  switch_msg = Twist()
  run = 1
  switch_msg.angular.x = run
  switch_msg.angular.y = work
  switch_pub.publish(switch_msg)

def stopCallBack():
  global run
  global work
  switch_msg = Twist()
  run = 0
  switch_msg.angular.x = run
  switch_msg.angular.y = work
  switch_pub.publish(switch_msg)

def ui():
  top = Tkinter.Tk()
  B_slam_on = Tkinter.Button(top, text ="start map",command = slamOnCallBack)
  B_slam_off = Tkinter.Button(top, text ="stop map",command = slamOffCallBack)
  B_navi_on = Tkinter.Button(top, text ="start navi",command = naviOnCallBack)
  B_navi_off = Tkinter.Button(top, text ="stop navi",command = naviOffCallBack)
  B_run = Tkinter.Button(top, text ="run",command = runCallBack)
  B_stop = Tkinter.Button(top, text ="stop",command = stopCallBack)
#  B_loop = Tkinter.Button(top, text ="loop",command = loopCallBack)

  B_slam_on.pack()
  B_slam_off.pack()
  B_navi_on.pack()
  B_navi_off.pack()
  B_run.pack()
  B_stop.pack()
#  B_loop.pack()
#  rospy.Timer(rospy.Duration(0.1), switch_pub_callback)
  top.mainloop()


if __name__ == "__main__":
  rospy.init_node('app_node', anonymous=True)
  switch_pub = rospy.Publisher('/car_goal_run', Twist, queue_size=1)

  t1 = threading.Thread(target=ui)
  t1.start()  
 

