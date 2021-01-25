#!/usr/bin/env python

import rospy
import math
import numpy as np 

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped


plan = []
track_plan = []
car_speed = 0
#car_state = []
pose_recv = False
goal_recv = False
run = False
class car_state:
  def __init__(self,x,y,yaw,v):
    self.x = x
    self.y = y
    self.yaw = yaw
    self.v = v
    
def twist_callback(msg):
  global car_speed
  car_speed = msg.linear.x
  
def switch_callback(msg):
  global run
  if msg.angular.x == 1:
    run = True
    print('1:SWITCH %d' %(run))
  else:
    run = False
    print('0:SWITCH %d' %(run))
  
def car_state_callback(msg):
  global state
  global pose_recv
  pose_recv = True
  state = car_state(x=msg.pose.pose.position.x,y=msg.pose.pose.position.y,yaw=msg.pose.pose.position.z,v=msg.twist.twist.linear.x)
#  car_state = [msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z,msg.twist.twist.linear.x]
 
#  print(car_state)
  
def path_callback(msg):
  global plan
  global goal_recv
  goal_recv = True
  
#  print(msg.poses)
#  print('O: %d' %(len(msg.poses)))
  del msg.poses[0 : 2]
#  print('C: %d' %(len(msg.poses)))
#  print(msg.poses)
  
#  print(len(msg.poses))

#  del x[:]
#  del y[:]
  x = []
  y = []
  for i in msg.poses:
    x.append(i.pose.position.x)
    y.append(i.pose.position.y)
  plan = zip(x,y)
#  del msg.poses[:]
#  print(plan)


def calc_target_index(cstate,plan,length):
  print('CURR:x:%.2f,y:%.2f' %(cstate.x,cstate.y))
  print(len(plan))
#  mid_np = np.array(plan)
#  mid_np_2f = np.round(mid_np,2)   
#  plan_new = list(mid_np_2f)   
#  print(plan_new)

#  for i in plan:
#      print('%.2f, %.2f' %(i[0],i[1]))

  ls = 0
  track_plan = []

  for i in plan:
    x1 = math.pow(i[0]-cstate.x,2)
    y1 = math.pow(i[1]-cstate.y,2)
    s1 = math.sqrt(x1+y1)
    if s1 >= 0.0:
      if s1 < ls:
        break
      print('S=%.2f' %(s1))
      track_plan.append(i)
    ls = s1
  
#  print('count=%d' %(len(track_plan)))   
#  print(track_plan)

  count = len(track_plan)
  print('count=%d' %(count))
  if count == 0:
    x = cstate.x
    y = cstate.y 
  for i in track_plan:
    count-=1
#    print('count=%.2f' %(count))
#    print('x:%.2f, y:%.2f' %(i[0],i[1]))
    x2 = math.pow(i[0]-cstate.x,2)
    y2 = math.pow(i[1]-cstate.y,2)
    s2 = math.sqrt(x2+y2)
    if s2 >= length or count == 0:
#      if s2 >= length: 
        x = i[0]
        y = i[1]
        break
#      else:
#        x = cstate.x
#        y = cstate.y

       
  path_msg = Path()
  path_msg.header.frame_id ='map'
#    path_msg.poses.clear()
#  print(track_plan)
  seq = 0
  for i in track_plan:
    track_msg = PoseStamped()
    track_msg.header.frame_id ='map'
    track_msg.header.seq = seq
    track_msg.pose.position.x = i[0]
    track_msg.pose.position.y = i[1]
    path_msg.poses.append(track_msg)
    seq+=1
  path_pub.publish(path_msg)
  
  return x, y 

def pure_pursuit_control(state, cx, cy):
   goal = math.atan2(cy - state.y, cx - state.x) #(math.pi/4)*0.5
   car = state.yaw
#   if goal < 0:
#     goal = 2 * math.pi + goal
#   if state.yaw < 0:
#     car = 2 * math.pi + state.yaw
   alpha = goal - car
   print('1goal:%.2f,curr:%.2f,diff:%.2f' %(goal,car,alpha))
   if alpha > math.pi:
     alpha = alpha - 2*math.pi
   if alpha < -math.pi:
     alpha = alpha + 2*math.pi
   print('2goal:%.2f,curr:%.2f,diff:%.2f' %(goal,car,alpha))
   vcar = np.array([math.cos(car), math.sin(car)])
   vgoal = np.array([math.cos(goal), math.sin(goal)])
#   lc = np.sqrt(vcar.dot(vcar))
#   lg = np.sqrt(vgoal.dot(vgoal))
#   dcos = vgoal.dot(vcar)/(lc*lg)
#   alpha = np.arccos(dcos)
   
   if alpha > math.pi/2:
     alpha = math.pi - alpha
   if alpha < -math.pi/2:
     alpha = -math.pi - alpha
   print('3goal:%.2f,curr:%.2f,diff:%.2f' %(goal,car,alpha))
   
   alpha = math.atan2(vgoal[1],vgoal[0])  - math.atan2(vcar[1],vcar[0])
   if alpha > math.pi:
     alpha = alpha-2*math.pi
   if alpha < -math.pi:
     alpha = alpha+2*math.pi
     
   print('4goal:%.2f,curr:%.2f,diff:%.2f' %(goal,car,alpha))
   L = 1.06
   k = 10
   Lf = k * 0.1
   delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)
#   print('alpha=%.2f,delta=%.2f' %(alpha,delta))
#   print('goal:%.2f,curr:%.2f,diff:%.2f' %(math.atan2(cy - state.y, cx - state.x),state.yaw,alpha))
   return delta
   
def speed_control(state, cx, cy):
  x2 = math.pow(cx-state.x,2)
  y2 = math.pow(cy-state.y,2)
  s2 = math.sqrt(x2+y2)
  
  alpha = math.atan2(cy - state.y, cx - state.x)# - state.yaw

  #dsin = math.sin(alpha) - math.sin(state.yaw)
  #dcos = math.cos(alpha) - math.cos(state.yaw)
  #dv = dsin/dcos;
  #dyaw = math.atan(dv);
  #if dcos<0:
  #  dyaw = dyaw + math.pi
  #else:
  #  dyaw = dyaw
  #print('dyaw=%.2f' %(dyaw))
  car = np.array([math.cos(state.yaw), math.sin(state.yaw)])
  goal = np.array([math.cos(alpha), math.sin(alpha)])
  lc = np.sqrt(car.dot(car))
  lg = np.sqrt(goal.dot(goal))
  dcos = car.dot(goal)/(lc*lg)
  #dy = np.dot(car, goal)
  dyaw = np.arccos(dcos)
  
  if dyaw > math.pi/2 or dyaw < -math.pi/2:
    speed = -1
  else:
    speed = 1
  print('sp%d,dyaw=%.2f' %(speed, dyaw))
  return speed   

def pure_pursuit_callback(event):
  global state
  global car_speed
  global run
#  print('x:%.2f' %(state.x))
#  print('y:%.2f' %(state.y))
  if pose_recv and goal_recv:
    gx,gy = calc_target_index(state,plan,1)
#    print('gx=%.2f,gy=%.2f' %(gx,gy))
  
    vth = pure_pursuit_control(state,gx,gy)
    vx = abs(car_speed) * speed_control(state,gx,gy)
    if vx == 0:
      vth = 0
    cmd_msg = Twist()
    run = True #False
    if run:
      print('RUN')
      cmd_msg.linear.x = vx
      cmd_msg.angular.z = vth
    else:
      print('STOP')
      cmd_msg.linear.x = 0
      cmd_msg.angular.z = 0
    
    pos_msg = PointStamped()
    pos_msg.header.frame_id ='map'
    pos_msg.point.x = gx
    pos_msg.point.y = gy
    

    
    cmd_pub.publish(cmd_msg)
    pos_pub.publish(pos_msg)

  
if __name__ == '__main__':
    try:
        rospy.init_node('pure_pursuit', anonymous = True)
        
        rospy.Subscriber('/mb_cmd', Twist, twist_callback)
        rospy.Subscriber('/luo_pos', Odometry, car_state_callback)
        rospy.Subscriber('/move_base/TebLocalPlannerROS/local_plan', Path, path_callback)
        rospy.Subscriber('/car_goal_run', Twist, switch_callback)
        cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        pos_pub = rospy.Publisher('/track_pos', PointStamped, queue_size=1)
        path_pub = rospy.Publisher('/track_path', Path, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), pure_pursuit_callback)
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
