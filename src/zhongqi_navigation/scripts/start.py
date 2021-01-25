#!/usr/bin/env python
import roslaunch
import rospy
from std_srvs.srv import Empty

rospy.init_node('navigation', anonymous=True)

launch_path = "/home/autolabor/zhongqi_ws/src/zhongqi_navigation/launch/zhongqi_navigation.launch"
#launch_path = "/home/autolabor/zhongqi_ws/src/zhongqi_navigation/launch/zhongqi_mpc_navigation.launch"

nav = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(nav)
nav_launch = roslaunch.parent.ROSLaunchParent(
    nav, [launch_path])
nav_launch.start()
rospy.loginfo("nav_started!!!")
rospy.sleep(1)

trans = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(trans)
trans_launch = roslaunch.parent.ROSLaunchParent(
    trans, ["/home/autolabor/zhongqi_ws/src/position_optimization/launch/trans.launch"])
trans_launch.start()
rospy.loginfo("trans_started!!!")
rospy.sleep(5)

def clear_timer(event):
  rospy.wait_for_service('/move_base/clear_costmaps')
  clear = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
  clear()
  rospy.loginfo("clear!!!")

rospy.Timer(rospy.Duration(20),clear_timer)

nav_launch.spin()
trans_launch.spin()

rospy.loginfo("stop!!!")
