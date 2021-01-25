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
front_left_obs = 0
front_right_obs = 0
front_middle_obs = 0
rear_left_obs = 0
rear_right_obs = 0
rear_middle_obs = 0
#obs_dist = [
#obs_dist = [
#  []
#]
def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
  if omega == 0 or v == 0:
#    return 0
    return omega
  radius = abs(v) / (omega*180/3.14)
#  print('cr%.2f' %(radius))
#  return math.atan(wheelbase / radius)
#  if v < 0:
#    omega = -omega
  return omega
  
  
def obstacle_detection(laser, min_dist, car_width, dev):
  brake = False
  l_obs = False
  r_obs = False
  brake_c = 0
  brake_o = 0
  lsteer = 0
  rsteer = 0
  count = 0
  sum_dist = []
  laser_ranges = []
  l_dist = 0
  m_dist = 0
  r_dist = 0
  cl = 0
  cr = 0
  cm = 0
  lowangle = math.atan2(min_dist, car_width/2)*180/math.pi # + 0.1
  num_data = len(laser.ranges)
#  scan_angle = abs(laser.angle_min) + abs(laser.angle_max)
  scan_angle = 180
  increment = laser.angle_increment
#  print('lowangle:%.2f-%.2f' %(lowangle, scan_angle - lowangle))
#  ranges_list = list(laser.ranges)
#  print(dev,num_data)
  for i in range(0,num_data): 
    l_dist = laser.ranges[i]
    i+=1
    count+=1
#    if l_dist == float('inf'):
#      continue
#      print(i,l_dist)
#      l_dist = sum_dist/count
    sum_dist.append(l_dist)
#    print((int((increment * i)*180/math.pi)))
    if (int(((increment * i)+(math.pi/2-abs(laser.angle_min)))*180/math.pi)%10) == 0 or i == num_data:
#      ave_dist = sum_dist/count

      if len(sum_dist)>2:
        ave_dist = min(sum_dist)
        laser_ranges.append(ave_dist)
#        print(dev,laser_ranges,len(laser_ranges),int(((increment * i)+(math.pi/2-abs(laser.angle_min)))*180/math.pi))
        sum_dist = []
        count = 0
  num_range = len(laser_ranges)
#  print(len(laser_ranges))
  for i in range(0,num_range): 
    laser_dist = laser_ranges[i]
#    angle = laser.angle_min + increment * i + math.pi/2
    angle = (10 * i) + ((math.pi/2-abs(laser.angle_min))*180/math.pi)
#    print('%s,angle:%.2f--dist:%.2f(%.2f)' %(dev,angle, laser_dist,math.sin(angle*math.pi/180) * laser_dist))
    if angle <= lowangle:
      if laser_dist < ((car_width/2) + min_dist*0.5): # + min_dist/2
#      if laser_dist < 0.35: 
        lsteer+=1
        l_dist = laser_dist
        cl+=1
#        print('LLL_OBS(<%.2f)=angle:%.2f dist:%.2f(%.2f)' %(lowangle,angle, laser_dist,math.sin(angle*math.pi/180) * laser_dist))
#      print('brake[%d]=LOW_angle:%.2f,laser_dist:%.2f,obs_dist:%.2f' %(i,angle, laser_dist,math.sin(increment * i) * laser_dist))
    elif angle >= (scan_angle - lowangle):
      if laser_dist < ((car_width/2) + min_dist*0.5): # + min_dist/2
#      if laser_dist < 0.35: 
        rsteer+=1
        r_dist = laser_dist
        cr+=1
#        print('RRR_OBS=angle:%.2f dist:%.2f(%.2f)' %(angle, laser_dist, math.sin(angle*math.pi/180) * laser_dist))
    elif angle > lowangle and angle < scan_angle - lowangle:
#      l_obs = r_obs = False
      if (math.sin(angle*math.pi/180) * laser_dist) >= min_dist:
        brake_o+=1 # = False
#        print('NO-brake[%d]=angle:%.2f,laser_dist:%.2f,obs_dist:%.2f' %(i,angle, laser_dist,math.sin(increment * i) * laser_dist))
      else:
#        print('brake[%d]=angle:%.2f,laser_dist:%.2f,obs_dist:%.2f' %(i,angle, laser_dist,math.sin(increment * i) * laser_dist))
        brake_c+=1 # = True
        m_dist = laser_dist 
        cm+=1
#        print('MMM_OBS=angle:%.2f dist:%.2f(%.2f)' %(angle, laser_dist, math.sin(angle*math.pi/180) * laser_dist))
    '''
    if laser_dist < 0.45:
#      print('dist:%.2f' %(math.sin(angle*math.pi/180) * laser_dist))
      if (math.sin(angle*math.pi/180) * laser_dist) >= min_dist:
        brake_o+=1
      else:
#        print('OBS angle:%.2f--dist:%.2f(%.2f)' %(angle, laser_dist,math.sin(angle*math.pi/180) * laser_dist))
        brake_c+=1
        if angle < 30:
          l_dist = laser_dist
          cl+=1
        elif angle > 150:
          r_dist = laser_dist
          cr+=1
        elif angle >= 30 and angle <= 150:
          m_dist = laser_dist 
          cm+=1
  '''
  if cl==0:
    l_dist = 0
  if cr==0:
    r_dist = 0
  if cm==0:
    m_dist = 0

##        print('MMM_OBS=angle:%.2f dist:%.2f' %(angle, laser_dist))
  if brake_c > 0:
      brake = True
#  if brake_c == 0:
#      brake = False
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
#  print('BBB:%d LLL:%d RRR:%d !!!' %(brake, l_obs, r_obs))
  return brake, l_dist, r_dist, m_dist

def front_laser_callback(data):
  global front_brake
  global wfl_steer
  global wfr_steer
  global front_left_obs
  global front_right_obs
  global front_middle_obs
  
  global rear_left_obs
  global rear_right_obs
  global rear_middle_obs
#obs_dist = [
  front_brake, front_left_obs, front_right_obs, front_middle_obs = obstacle_detection(data,0.17, 0.84, 'front')#0.18 0.83
#  print('                             FRONT_BRAKE:%d !!!' %(front_brake))
  
  msg = Twist()
  msg.linear.x = front_left_obs
  msg.linear.y = front_right_obs
  msg.linear.z = front_middle_obs
  msg.angular.x = rear_left_obs
  msg.angular.y = rear_right_obs
  msg.angular.z = rear_middle_obs
  obs_pub.publish(msg)

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
  global rear_left_obs
  global rear_right_obs
  global rear_middle_obs
#obs_dist = [
  rear_brake, rear_right_obs, rear_left_obs, rear_middle_obs = obstacle_detection(data,0.17, 0.84, 'rear')#0.18
#  print('                             REAR_BRAKE:%d !!!' %(rear_brake))

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
  global goal_v
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
  #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!#
  if front_brake and data.linear.x > 0:
#    if curr_v > 0 or (curr_v == 0 and data.linear.x > 0):
##      gearcmd = gearcmd|4
##      out = 0
      print('front_brake:%d' %(gearcmd))
  if rear_brake and data.linear.x < 0:
#    if curr_v > 0 or (curr_v == 0 and data.linear.x > 0):
##      gearcmd = gearcmd|4
##      out = 0
      print('rear_brake:%d' %(gearcmd))
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
#    out = 0
  laststeer = steering
  '''
  if out > 0.5 and curr_v == 0:
    zerospeed+=1
    if zerospeed > 15:
      gearcmd = 0
      out = 0
      print('ZERO: %d' %(zerospeed))
  else:
    zerospeed = 0
  '''
##################
#  if curr_v <= 0.02 and out > 0.8:# and goal_v < 0.1:
#      gearcmd = 0
#      out = 0.3
  msg = AckermannDriveStamped()
  msg.header.stamp = rospy.Time.now()
  msg.header.frame_id = frame_id
  msg.drive.steering_angle = steering #*0.7
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
  global pid_pub
  curr_v = abs(data.twist.twist.linear.z)
  goal_v = abs(cv)
  error = (goal_v - curr_v)
  sum_err = error + last_err + last_err1
  diff_err = last_err - error
#  print('E: %.2f,S: %.2f,D: %.2f' %(error,sum_err,diff_err))
#  if abs(error)*5<0.02:
#    out = out
#  else:
  result = kp * (error - last_err) + ki * error + kd * (error - 2 * last_err + last_err1)

  out += result
#  print('G: %.2f,C: %.2f,R: %.2f, O: %.2f' %(goal_v,curr_v,result,out))
  flag_v = last_v * cv
#  print('E: %.2f', last_v )

#  if curr_v <= 0.2 and out > 0.6:
#      gearcmd = 0
#      out = 0.3
  if out < 0 or flag_v < 0  or gearcmd != gearback:
#    print('out:%.2f, flag_v:%.2f,(%d,%d) last:%.2f,cv:%.2f' %(out,flag_v,gearcmd, gearback,last_v, cv))
    out = 0
    
  if out >= 1:
    out = 1
  
  if rospy.get_time()-lasttime > 0.2:
    out = 0
#    print('timeout:%.2f' %(rospy.get_time()-lasttime))
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
    ki = client.get_configuration()["i"]
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
    wheelbase = rospy.get_param('~wheelbase', 1.06)
    frame_id = rospy.get_param('~frame_id', 'odom')
    
    pub = rospy.Publisher(ackermann_cmd_topic, AckermannDriveStamped, queue_size=1)
    obs_pub = rospy.Publisher('obs_direction', Twist, queue_size=1)
    pid_pub = rospy.Publisher('pid_db', Twist, queue_size=1)
    
    rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)
    rospy.Subscriber("odom", Odometry, odom_callback, queue_size=1)
    rospy.Subscriber("vehicle_info", vehicle_info, vehicle_callback, queue_size=1)
    rospy.Subscriber('scan_1', LaserScan, front_laser_callback)
    rospy.Subscriber('scan_2', LaserScan, rear_laser_callback)
    
    rospy.loginfo("Node 'cmd_vel_to_ackermann_drive' started.\nListening to %s, publishing to %s. Frame id: %s, wheelbase: %f", "/cmd_vel", ackermann_cmd_topic, frame_id, wheelbase)
  #  timer(5)
    srv = Server(PidConfig, cfg_callback)
  #  pid_init(9.0,3.5,1.0)
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass

