#!/usr/bin/env python

import rospy
import math
import numpy as np
import PyKDL
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from dynamic_reconfigure.server import Server
from zhongqi_driver.cfg import PidConfig

plan = []
track_plan = []
car_speed = 0
pose_recv = False
goal_recv = False
run = False
track_length = 0.3
ppk = 10
timezero = 0
vzero = False
plan_recv_ctrl = 0
obs_dist = np.zeros((2, 1))

class car_state:
    def __init__(self, x, y, yaw, v):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    #  print(rot.GetRPY())
    return rot.GetRPY()[2]


def twist_callback(msg):
    global car_speed
    global timezero
    global vzero
    if msg.linear.x == 0:
      timezero+=1
      if timezero >= 2:
        car_speed = msg.linear.x
      else:
        car_speed = 0.1
    else:
      car_speed = msg.linear.x
      timezero = 0


def obs_direction_callback(msg):
  global obs_dist
  obs_dist = [[-msg.linear.x, msg.linear.z, msg.linear.y],  [-msg.angular.x, msg.angular.z, msg.angular.y]]
#  print(obs_dist)

def switch_callback(msg):
    global run
    if msg.angular.x == 1:
        run = True
#        print("1:SWITCH %d" % (run))
    else:
        run = False
#        print("0:SWITCH %d" % (run))


def car_state_callback(msg):
    global state
    global pose_recv
    pose_recv = True
    z=quat_to_angle(msg.pose.pose.orientation)
    state = car_state(
        x=msg.pose.pose.position.x,
        y=msg.pose.pose.position.y,
        yaw=z,
        v=0
#        x=msg.pose.pose.position.x,
#        y=msg.pose.pose.position.y,
#        yaw=msg.pose.pose.position.z,
#        v=msg.twist.twist.linear.x,
    )


def path_callback(msg):
    global plan
    global goal_recv
    global plan_recv_ctrl
    goal_recv = True
    plan_recv_ctrl+=1
#    del msg.poses[0: 1]

    x = []
    y = []
    yaw = []
    for i in msg.poses:
        q2y = quat_to_angle(i.pose.orientation)
        yaw.append(q2y)
        x.append(i.pose.position.x)
        y.append(i.pose.position.y)
    plan = zip(x, y, yaw)

'''
    num = plan_recv_ctrl%5
    print('MUN:%d,CTRL:%d' %(plan_recv_ctrl,num))
    if num == 1:
        print('RECV!!!')
        for i in msg.poses:
            q2y = quat_to_angle(i.pose.orientation)
            yaw.append(q2y)
            x.append(i.pose.position.x)
            y.append(i.pose.position.y)
        plan = zip(x, y, yaw)
    if plan_recv_ctrl >= 100:
        plan_recv_ctrl = 0
'''

def calc_target_index(cstate, plan, length):
    goal = {"x": 0, "y": 0, "yaw": 0}
    last_point = False
    #  mid_np = np.array(plan)
    #  mid_np_2f = np.round(mid_np,2)
    #  plan_new = list(mid_np_2f)
    #  print(plan_new)

    #  for i in plan:
    #      print('%.2f, %.2f' %(i[0],i[1]))
#    print("plan_len:%d" % (len(plan)))
    #    print('print_plan:\n'+'\n'.join(str(i) for i in plan))
    ls = 0
    track_plan = []
#    print(plan[0])
#    print("S:")
    for i in plan:
        x1 = math.pow(i[0] - cstate.x, 2)
        y1 = math.pow(i[1] - cstate.y, 2)
        s1 = math.sqrt(x1 + y1)
#        print("%.2f" % (s1))

        #        if s1 >= 0.0:
        #            if s1 < ls:
        #                break
        #            track_plan.append(i)

        if s1 < ls:
            if len(track_plan) > 1 and ls > 0.1:
                #                print('LEN_OK:%.2f==%d' %(s1,len(track_plan)))
                ls = 0
                break
            else:
                track_plan = []
#                print('LEN_NO:%.2f==%.2f' %(s1,ls))
                ls = 0
        else:
            track_plan.append(i)
            ls = s1

    #    print('print_track_plan:\n'+'\n'.join(str(i) for i in track_plan))

    count = len(track_plan)
#    print('track_plan:%d' %(len(track_plan)))
#    print('x:%.2f, y:%.2f' %(cstate.x,cstate.y))
#    print(track_plan)  
       
    if count == 0:
#        goal["x"] = cstate.x
#        goal["y"] = cstate.y
#        goal["yaw"] = cstate.yaw
        last_point = True
        print('LAST_POINT!') 
    else:
        last_point = False
    n=0
    for i in track_plan:
        count -= 1
        n+=1
        #    print('count=%.2f' %(count))
        #    print('x:%.2f, y:%.2f' %(i[0],i[1]))
        x2 = math.pow(i[0] - cstate.x, 2)
        y2 = math.pow(i[1] - cstate.y, 2)
        s2 = math.sqrt(x2 + y2)
#        if s2 >= length or count == 0:
        if n >= 5 or count == 0:
#            print(n)
            goal["x"] = i[0]
            goal["y"] = i[1]
            goal["yaw"] = i[2]
            n=0
            break

    path_msg = Path()
    path_msg.header.frame_id = "map"
    #  print(track_plan)
    seq = 0
    for i in track_plan:
        track_msg = PoseStamped()
        track_msg.header.frame_id = "map"
        track_msg.header.seq = seq
        track_msg.pose.position.x = i[0]
        track_msg.pose.position.y = i[1]
        path_msg.poses.append(track_msg)
        seq += 1
    path_pub.publish(path_msg)

    return goal


def pure_pursuit_control(state, tgoal):
    global ppk
    goal = math.atan2(tgoal["y"] - state.y, tgoal["x"] - state.x)  # (math.pi/4)*0.5
    car = state.yaw
    #   if goal < 0:
    #     goal = 2 * math.pi + goal
    #   if state.yaw < 0:
    #     car = 2 * math.pi + state.yaw
    alpha = goal - car
    #    print('1goal:%.2f,curr:%.2f,diff:%.2f' %(goal,car,alpha))
    if alpha > math.pi:
        alpha = alpha - 2 * math.pi
    if alpha < -math.pi:
        alpha = alpha + 2 * math.pi
    if alpha > math.pi / 2:
        alpha = math.pi - alpha
    if alpha < -math.pi / 2:
        alpha = -math.pi - alpha
#    print('2goal:%.2f,curr:%.2f,diff:%.2f' %(goal,car,alpha))
    vcar = np.array([math.cos(car), math.sin(car)])
    vgoal = np.array([math.cos(tgoal["yaw"]), math.sin(tgoal["yaw"])])
    #   vgoal = np.array([math.cos(goal), math.sin(goal)])
    #   lc = np.sqrt(vcar.dot(vcar))
    #   lg = np.sqrt(vgoal.dot(vgoal))
    #   dcos = vgoal.dot(vcar)/(lc*lg)
    #   alpha = np.arccos(dcos)

    if alpha > math.pi / 2:
        alpha = math.pi - alpha
    if alpha < -math.pi / 2:
        alpha = -math.pi - alpha
    #    print('3goal:%.2f,curr:%.2f,diff:%.2f' %(goal,car,alpha))

    alpha1 = math.atan2(vgoal[1], vgoal[0]) - math.atan2(vcar[1], vcar[0])
#    print('4goal:%.2f,curr:%.2f,diff:%.2f' % (tgoal['yaw'], car, alpha1))
    if alpha1 > math.pi:
#        print('>>>>>>>>>>')
        alpha1 = alpha1 - 2 * math.pi
    if alpha1 < -math.pi:
#        print('<<<<<<<<<<')
        alpha1 = alpha1 + 2 * math.pi
    direc = direction_judge(state, tgoal)
    if direc > 0:
        alpha1 = alpha1
    else:
        alpha1 = -alpha1
#    print('4goal:%.2f,curr:%.2f,diff:%.2f' % (tgoal['yaw'], car, alpha1))

    sinv = 0.5 * math.sin(abs(alpha)) + 0.5 * math.sin(abs(alpha1))
    cosv = 0.5 * math.cos(abs(alpha)) + 0.5 * math.cos(abs(alpha1))
    difv = sinv/cosv
    alpha2 = math.atan(difv) * np.sign(alpha1)
#    print('alpha:%.2f,alpha2:%.2f,alpha1:%.2f' % (alpha, alpha2, alpha1))  

    L = 1.06
    k = ppk #* 5
    #    print('K=%.2f' %(k))
    Lf = k * 0.1
    delta = math.atan2(2.0 * L * math.sin(alpha2) / Lf, 1.0)
    #   print('alpha=%.2f,delta=%.2f' %(alpha,delta))
    # print('goal:%.2f,curr:%.2f,diff:%.2f' %(math.atan2(cy - state.y, cx -
    # state.x),state.yaw,alpha))
    return delta


def direction_judge(state, goal):
    x2 = math.pow(goal["x"] - state.x, 2)
    y2 = math.pow(goal["y"] - state.y, 2)
    s2 = math.sqrt(x2 + y2)
    alpha = math.atan2(goal["y"] - state.y, goal["x"] - state.x)
    car = np.array([math.cos(state.yaw), math.sin(state.yaw)])
    goal = np.array([math.cos(alpha), math.sin(alpha)])
    lc = np.sqrt(car.dot(car))
    lg = np.sqrt(goal.dot(goal))
    dcos = car.dot(goal) / (lc * lg)
    eps = 1e-6
    if 1.0 < dcos < 1.0 + eps:
        dcos = 1.0
    elif -1.0 - eps < dcos < -1.0:
        dcos = -1.0
    dyaw = np.arccos(dcos)
    if dyaw > math.pi / 2 or dyaw < -math.pi / 2:
        direction = -1
    else:
        direction = 1
    #  print('sp%d,dyaw=%.2f' %(direction, dyaw))
    return direction


def obstacle_direction(state, goal, obs_dist, direc):
  new_goal = {"x": 0, "y": 0, "yaw": 0}
  front_obs = 0
  rear_obs = 0
  front_lenth = 1.28
  rear_lenth = -0.31
  brake = 0
  for i in range(0, len(obs_dist)):
    for j in range(0, len(obs_dist[i])):
      if i==0:
        if j==0:
          if obs_dist[i][j] == 0:
            front_obs=front_obs&~(1<<2)
          else:
            front_obs=front_obs|(1<<2)
        elif j==1:
          if obs_dist[i][j] == 0:
            front_obs=front_obs&~(1<<1)
          else:
            front_obs=front_obs|(1<<1)
        elif j == 2:
          if obs_dist[i][j] == 0:
            front_obs=front_obs&~1
          else:
            front_obs=front_obs|1
      elif i==1:
        if j==0:
          if obs_dist[i][j] == 0:
            rear_obs=rear_obs&~(1<<2)
          else:
            rear_obs=rear_obs|(1<<2)
        elif j==1:
          if obs_dist[i][j] == 0:
            rear_obs=rear_obs&~(1<<1)
          else:
            rear_obs=rear_obs|(1<<1)
        elif j == 2:
          if obs_dist[i][j] == 0:
            rear_obs=rear_obs&~1
          else:
            rear_obs=rear_obs|1
#  print('obs:', bin(front_obs),bin(rear_obs))
  if front_obs+rear_obs == 0:
    beake = 0
    return goal, brake 
  # 4,2,1,6,3,5,7
  front_obs_dict = {'1': obs_dist[0][2], 
                    '2': obs_dist[0][1], 
                    '3': obs_dist[0][2], 
                    '4': obs_dist[0][0], 
                    '5': obs_dist[0][0], 
                    '6': obs_dist[0][0]}
  rare_obs_dict = { '1': obs_dist[1][2], 
                    '2': obs_dist[1][1], 
                    '3': obs_dist[1][2], 
                    '4': obs_dist[1][0], 
                    '5': obs_dist[1][0], 
                    '6': obs_dist[1][0]}

#  print rare_obs_dict[str(rear_obs)]
  if (front_obs != 0) and (direc == 1):
    if front_obs == 4 or front_obs == 1 or front_obs == 3 or front_obs == 6:
      print('x:%.2f,y:%.2f' %(front_lenth, front_obs_dict[str(front_obs)]))
      obs_x = front_lenth*math.cos(state.yaw)-2*front_obs_dict[str(front_obs)]*math.sin(state.yaw)
      obs_y = 2*front_obs_dict[str(front_obs)]*math.cos(state.yaw)+front_lenth*math.sin(state.yaw)

#      obs_x = 3*math.cos(-math.pi/4)-5*math.sin(-math.pi/4)
#      obs_y = 5*math.cos(-math.pi/4)+3*math.sin(-math.pi/4)
      print('x:%.2f,y:%.2f,yaw:%.2f' %(obs_x, obs_y, state.yaw)) 
      obs_point = np.array([obs_x, obs_y])
      print('fobs', obs_point)
      goal_point = np.array([goal["x"] - state.x, goal["y"] - state.y])
      print('goal', goal_point)
      tem_goal = 0.5*obs_point + goal_point
      print(tem_goal)
      new_goal["x"] = tem_goal[0]+state.x
      new_goal["y"] = tem_goal[1]+state.y
      new_goal["yaw"] = goal["yaw"]
   
    elif front_obs == 7: # or front_obs == 2:
      brake = 1
      
  elif (rear_obs != 0) and (direc == -1):
    if rear_obs == 4 or rear_obs == 1 or rear_obs == 3 or rear_obs == 6:
      obs_x = rear_lenth*math.cos(state.yaw)-2*rare_obs_dict[str(rear_obs)]*math.sin(state.yaw)
      obs_y = 2*rare_obs_dict[str(rear_obs)]*math.cos(state.yaw)+rear_lenth*math.sin(state.yaw)
      obs_point = np.array([obs_x, obs_y])
      print('robs', obs_point)
      goal_point = np.array([goal["x"] - state.x, goal["y"] - state.y])
      print('goal', goal_point)
      tem_goal = 0.5*obs_point + goal_point
      print(tem_goal)
      new_goal["x"] = tem_goal[0]+state.x
      new_goal["y"] = tem_goal[1]+state.y
      new_goal["yaw"] = goal["yaw"]

    elif rear_obs == 7: # or rear_obs == 2:
      brake = 1
  else:
    beake = 0
    return goal, brake 
  return new_goal, brake  


def pure_pursuit_callback(event):
    global state
    global car_speed
    global run
    global track_length
    global obs_dist
    brake = 0
    if pose_recv and goal_recv:
        #        print('track_length=%.2f' %(track_length))
        goal = calc_target_index(state, plan, track_length)
#        print('C=x:%.2f,y:%.2f,G=x:%.2f,y:%.2f' %(state.x,state.y,goal['x'],goal['y']))
        direc = direction_judge(state, goal)
        goal, brake = obstacle_direction(state, goal, obs_dist, direc)

        vx = abs(car_speed) * direc
        #        vx = 0.1 * direction_judge(state, goal)
        vth = pure_pursuit_control(state, goal)
#        print("vx:%.2f,vth:%.2f" % (vx, vth))
        
        if vx == 0:
            vth = 0
        cmd_msg = Twist()
        run = True  # False
        if run:
            #      print('RUN')
            cmd_msg.linear.x = vx
            cmd_msg.angular.x = brake
            cmd_msg.angular.z = vth
        else:
            #      print('STOP')
            cmd_msg.linear.x = 0
            cmd_msg.angular.x = 1
            cmd_msg.angular.z = 0

        pos_msg = PointStamped()
        pos_msg.header.frame_id = "map"
        pos_msg.point.x = goal["x"]
        pos_msg.point.y = goal["y"]

        cmd_pub.publish(cmd_msg)
        pos_pub.publish(pos_msg)


def cfg_callback(config, level):
    global track_length
    global ppk
    #  global kd
    track_length = config.p
    ppk = config.i
    print("len:%.2f,k:%.2f" % (track_length, ppk))
    #  kd = config.d
    return config


if __name__ == "__main__":
    try:
        rospy.init_node("pure_pursuit", anonymous=True)

        rospy.Subscriber("/mb_cmd", Twist, twist_callback)
        rospy.Subscriber("/obs_direction", Twist, obs_direction_callback)
#        rospy.Subscriber("/luo_pos", Odometry, car_state_callback)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, car_state_callback)
        rospy.Subscriber(
            "/move_base/TebLocalPlannerROS/local_plan", Path, path_callback
        )
        rospy.Subscriber("/car_goal_run", Twist, switch_callback)
        cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        pos_pub = rospy.Publisher("/track_pos", PointStamped, queue_size=1)
        path_pub = rospy.Publisher("/track_path", Path, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), pure_pursuit_callback)

#        srv = Server(PidConfig, cfg_callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
