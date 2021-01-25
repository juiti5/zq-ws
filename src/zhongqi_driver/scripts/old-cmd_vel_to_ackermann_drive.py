#!/usr/bin/env python

# Author: christoph.roesmann@tu-dortmund.de

import rospy, math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from zhongqi_driver.msg import vehicle_info

import matplotlib.pyplot as plt
import time
from dynamic_reconfigure.server import Server
from zhongqi_driver.cfg import PidConfig
from sensor_msgs.msg import LaserScan

cv = 0
sum_err = 0
last_err = 0
last_err1 = 0
diff_err = 0
curr_v = 0
goal_v = 0
last_v = 0
error = 0
out = 0
kp = 0
ki = 0
kd = 0
gearback = 0
gearcmd = 0
laststeer = 0
zerospeed = 0
lasttime = 0 
front_brake = False
wfl_steer = False
wfr_steer = False
rear_brake = False
wrl_steer = False
wrr_steer = False
#obs_dist = [
#  []
#]
def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
  if omega == 0 or v == 0:
#    return 0
    return omega
  radius = abs(v) / omega
#  return math.atan(wheelbase / radius)
#  if v < 0:
#    omega = -omega
  return omega
  
  
def obstacle_detection(laser, min_dist, car_width):
  brake = False
  l_obs = False
  r_obs = False
  brake_c = 0
  brake_o = 0
  lsteer = 0
  rsteer = 0
  lowangle = math.atan2(min_dist, car_width) + 0.1
  num_data = len(laser.ranges)
  scan_angle = abs(laser.angle_min) + abs(laser.angle_max)
  increment = laser.angle_increment
#  print('lowangle:%.2f-%.2f' %(lowangle, scan_angle - lowangle))
#  ranges_list = list(laser.ranges)
#  print(type(ranges_list))
  for i in range(0,num_data): 
    laser_dist = laser.ranges[i]
    i+=1
    if laser_dist == float('inf'):
      continue
    angle = laser.angle_min + increment * i + math.pi/2
#    print('angle:%.2f--%.2f' %(angle, increment*i))
    if angle <= lowangle:
      if laser_dist < ((car_width/2) + min_dist/2):
        lsteer+=1
        print('L_OBS')
#      print('brake[%d]=LOW_angle:%.2f,laser_dist:%.2f,obs_dist:%.2f' %(i,angle, laser_dist,math.sin(increment * i) * laser_dist))
    elif angle >= (scan_angle - lowangle):
      if (laser_dist < ((car_width/2) + min_dist/2)):
        rsteer+=1
        print('R_OBS')
    elif angle > lowangle and angle < scan_angle - lowangle:
#      l_obs = r_obs = False
      if (math.sin(angle) * laser_dist) >= min_dist:
        brake_o+=1 # = False
#        print('NO-brake[%d]=angle:%.2f,laser_dist:%.2f,obs_dist:%.2f' %(i,angle, laser_dist,math.sin(increment * i) * laser_dist))
      else:
#        print('brake[%d]=angle:%.2f,laser_dist:%.2f,obs_dist:%.2f' %(i,angle, laser_dist,math.sin(increment * i) * laser_dist))
        brake_c+=1 # = True
  if brake_c > 0:
      brake = True
  if brake_c == 0:
      brake = False
  if lsteer > 0:
      l_obs = True
      brake = True
  elif lsteer == 0:
      l_obs = False
  if rsteer > 0:
      r_obs = True
      brake = True
  elif rsteer == 0:
      r_obs = False
  print('BBB:%d LLL:%d RRR:%d !!!' %(brake, l_obs, r_obs))
  return brake, l_obs, r_obs

def front_laser_callback(data):
  global front_brake
  global wfl_steer
  global wfr_steer
  front_brake, wfl_steer, wfr_steer = obstacle_detection(data,0.1, 0.83)
  print('FRONT_BRAKE:%d !!!' %(front_brake))
'''
  front_brake_c = 0
  front_brake_o = 0
  lsteer = 0
  rsteer = 0
  min_dist = 0.15
  width = 0.83
  lowangle = math.atan2(min_dist, width) + 0.1
  num_data = len(data.ranges)
  scan_angle = abs(data.angle_min) + abs(data.angle_max)
  increment = data.angle_increment
  print('lowangle:%.2f-%.2f' %(lowangle, scan_angle - lowangle))
#  ranges_list = list(data.ranges)
#  print(type(ranges_list))
  for i in range(0,num_data): 
    laser_dist = data.ranges[i]
    i+=1
    if laser_dist == float('inf'):
      continue
    angle = data.angle_min + increment * i + math.pi/2
#    print('angle:%.2f--%.2f' %(angle, increment*i))
    if angle <= lowangle:
      if laser_dist < ((width/2) + min_dist):
        lsteer+=1
        print('L_OBS')
#      print('front_brake[%d]=LOW_angle:%.2f,laser_dist:%.2f,obs_dist:%.2f' %(i,angle, laser_dist,math.sin(increment * i) * laser_dist))
    elif angle >= (scan_angle - lowangle):
      if (laser_dist < ((width/2) + min_dist)):
        rsteer+=1
        print('R_OBS')
    elif angle > lowangle and angle < scan_angle - lowangle:
#      wl_steer = wr_steer = False
      if (math.sin(angle) * laser_dist) >= min_dist:
        front_brake_o+=1 # = False
#        print('NO-front_brake[%d]=angle:%.2f,laser_dist:%.2f,obs_dist:%.2f' %(i,angle, laser_dist,math.sin(increment * i) * laser_dist))
      else:
#        print('front_brake[%d]=angle:%.2f,laser_dist:%.2f,obs_dist:%.2f' %(i,angle, laser_dist,math.sin(increment * i) * laser_dist))
        front_brake_c+=1 # = True
  if front_brake_c > 0:
      front_brake = True
      print('front_brake!!!')
  if front_brake_c == 0:
      print('NOO_front_brake!!!')
      front_brake = False
  if lsteer > 0:
      wl_steer = True
  elif lsteer == 0:
      wl_steer = False
  if rsteer > 0:
      wr_steer = True
  elif rsteer == 0:
      wr_steer = False
'''
def rear_laser_callback(data):
  global rear_brake
  global wrl_steer
  global wrr_steer
  rear_brake, wrr_steer, wrl_steer = obstacle_detection(data,0.1, 0.83)
  print('REAR_BRAKE:%d !!!' %(rear_brake))

def cmd_callback(data):
  global wheelbase
  global ackermann_cmd_topic
  global frame_id
  global pub
  global out
  global cv 
  global gearcmd
  global gearcmd
  global gearback
  global laststeer
  global zerospeed 
  global curr_v
  global lasttime
  global front_brake
  global wfl_steer
  global wfr_steer
  global rear_brake
  global wrl_steer
  global wrr_steer
  
  lasttime = rospy.Time.now().to_sec() 
  cv = data.linear.x
  
  if data.linear.x == 0:
    gearcmd = 0
  elif data.linear.x > 0:
    gearcmd = 1
  elif data.linear.x < 0:
    gearcmd = 2
#    steering = -steering

  if front_brake and data.linear.x > 0:
#    if curr_v > 0 or (curr_v == 0 and data.linear.x > 0):
#      gearcmd = gearcmd|4
      out = 0

  if rear_brake and data.linear.x < 0:
#    if curr_v > 0 or (curr_v == 0 and data.linear.x > 0):
#      gearcmd = gearcmd|4
      out = 0

  if (data.angular.x == 1):
    gearcmd = gearcmd|4
#  print('front_brake:%d, gear:%d' %((gearcmd>>2),gearcmd&3))
  
  steering = convert_trans_rot_vel_to_steering_angle(cv, data.angular.z, wheelbase)

  '''
  if (wfl_steer and steering > 0) or (wfr_steer and steering < 0):
     print('STEER!!!')
     steering = -steering
  if (wrl_steer and steering > 0) or (wrr_steer and steering < 0):
     print('STEER!!!')
     steering = -steering
  '''
  if gearcmd != gearback:
    steering = laststeer
  laststeer = steering
  
  if out > 0.5 and curr_v == 0:
    zerospeed+=1
    if zerospeed > 15:
      gearcmd = 0
      out = 0
#    print('ZERO: %d' %(zerospeed))
  else:
    zerospeed = 0
  
  msg = AckermannDriveStamped()
  msg.header.stamp = rospy.Time.now()
  msg.header.frame_id = frame_id
  msg.drive.steering_angle = steering
  msg.drive.speed = out
  msg.drive.jerk = gearcmd

  pub.publish(msg)

def vehicle_callback(data):
  global gearback
  gearback = data.gear

def odom_callback(data):
  global cv 
  global sum_err
  global last_err
  global last_err1
  global diff_err
  global curr_v
  global goal_v
  global last_v
  global error
  global out
  global kp
  global ki  
  global kd
  global gearcmd
  global gearback
  global lasetime
  curr_v = abs(data.twist.twist.linear.z)
  goal_v = abs(cv)
  error = (goal_v - curr_v)/5
  sum_err = error + last_err + last_err1
  diff_err = last_err - error
#  print('E: %.2f,S: %.2f,D: %.2f' %(error,sum_err,diff_err))

  result = kp * (error - last_err) + ki * error + kd * (error - 2 * last_err + last_err1)

#  print('O: %.2f,O: %.2f,R: %.2f, O: %.2f' %(goal_v,curr_v,result,out))
  out += result
  flag_v = last_v * cv
#  print('E: %.2f', flag_v )
  if curr_v <= 0.2 and out > 0.5:
      out = 0.35
  if out <= 0 or flag_v <= 0  or gearcmd != gearback:
    out = 0
  if out >= 1:
    out = 1
  
  if rospy.get_time()-lasttime > 0.2:
    out = 0
#  print('now:%.2f, lasttime:%.2f' %(rospy.get_time(),lasttime))
  last_v = cv

  msg = Twist()
  msg.linear.x = curr_v
  msg.linear.y = goal_v
  msg.linear.z = out
  pid_pub.publish(msg)
#  print('P: %.2f,I: %.2f,D: %.2f' %(kp,ki,kd))
#  print('G: %.2f,C: %.2f,O: %.2f' %(goal_v,curr_v,out))
#  print('L: %.2f,E: %.2f,S: %.2f' %(last_err,error,sum_err))
  last_err1 = last_err
  last_err = error

def timer(n):
  while True:
    kp = client.get_configuration()["p"]
    ki =client.get_configuration()["i"]
    kd = client.get_configuration()["d"]
    time.sleep(n)

def cfg_callback(config, level):
  global kp
  global ki
  global kd
  kp = config.p
  ki = config.i
  kd = config.d
  
  print('P: %.2f,I: %.2f,D: %.2f' %(kp,ki,kd))
  
  return config
  
def pid_init(p,i,d):
  global kp
  global ki
  global kd
  kp = p
  ki = i
  kd = d
  
if __name__ == '__main__': 
  try:
    
    rospy.init_node('cmd_vel_to_ackermann_drive')
        
    twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel') 
    ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/ackermann_cmd')
    wheelbase = rospy.get_param('~wheelbase', 1.0)
    frame_id = rospy.get_param('~frame_id', 'odom')
    
    rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)
    
    rospy.Subscriber("odom", Odometry, odom_callback, queue_size=1)
    rospy.Subscriber("vehicle_info", vehicle_info, vehicle_callback, queue_size=1)
    rospy.Subscriber('scan_1', LaserScan, front_laser_callback)
    rospy.Subscriber('scan_2', LaserScan, rear_laser_callback)
    
    pub = rospy.Publisher(ackermann_cmd_topic, AckermannDriveStamped, queue_size=1)
    
    pid_pub = rospy.Publisher('pid_db', Twist, queue_size=1)
    
    rospy.loginfo("Node 'cmd_vel_to_ackermann_drive' started.\nListening to %s, publishing to %s. Frame id: %s, wheelbase: %f", "/cmd_vel", ackermann_cmd_topic, frame_id, wheelbase)
  #  timer(5)
    srv = Server(PidConfig, cfg_callback)
  #  pid_init(9.0,3.5,1.0)
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass

