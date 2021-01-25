#!/usr/bin/env python

import rospy
import math
import numpy as np
import PyKDL
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from dynamic_reconfigure.server import Server
from zhongqi_driver.cfg import PidConfig

plan = []
track_plan = []
car_speed = 0
pose_recv = False
goal_recv = False
run = False
track_length = 0.5
ppk = 20


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
    car_speed = msg.linear.x


def switch_callback(msg):
    global run
    if msg.angular.x == 1:
        run = True
        print('1:SWITCH %d' % (run))
    else:
        run = False
        print('0:SWITCH %d' % (run))


def car_state_callback(msg):
    global state
    global pose_recv
    pose_recv = True
    state = car_state(
        x=msg.pose.pose.position.x,
        y=msg.pose.pose.position.y,
        yaw=msg.pose.pose.position.z,
        v=msg.twist.twist.linear.x)


def path_callback(msg):
    global plan
    global goal_recv
    goal_recv = True

    del msg.poses[0: 1]

    x = []
    y = []
    yaw = []
    for i in msg.poses:
        q2y = quat_to_angle(i.pose.orientation)
        yaw.append(q2y)
        x.append(i.pose.position.x)
        y.append(i.pose.position.y)
    plan = zip(x, y, yaw)


def calc_target_index(cstate, plan, length):
    goal = {'x': 0, 'y': 0, 'yaw': 0}
#  mid_np = np.array(plan)
#  mid_np_2f = np.round(mid_np,2)
#  plan_new = list(mid_np_2f)
#  print(plan_new)

#  for i in plan:
#      print('%.2f, %.2f' %(i[0],i[1]))
    print('plan_len:%d' %(len(plan)))
#    print('print_plan:\n'+'\n'.join(str(i) for i in plan))
    ls = 0
    track_plan = []
    print('S:')
    for i in plan:
        x1 = math.pow(i[0] - cstate.x, 2)
        y1 = math.pow(i[1] - cstate.y, 2)
        s1 = math.sqrt(x1 + y1)
        print('%.2f' %(s1))
        
#        if s1 >= 0.0:
#            if s1 < ls:
#                break
#            track_plan.append(i)
        
        if s1 < ls:
            if ls >= 0.15:
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
    if count == 0:
        goal['x'] = cstate.x
        goal['y'] = cstate.y
        goal['yaw'] = cstate.yaw
    for i in track_plan:
        count -= 1
#    print('count=%.2f' %(count))
#    print('x:%.2f, y:%.2f' %(i[0],i[1]))
        x2 = math.pow(i[0] - cstate.x, 2)
        y2 = math.pow(i[1] - cstate.y, 2)
        s2 = math.sqrt(x2 + y2)
        if s2 >= length or count == 0:
            goal['x'] = i[0]
            goal['y'] = i[1]
            goal['yaw'] = i[2]

            break

    path_msg = Path()
    path_msg.header.frame_id = 'map'
#  print(track_plan)
    seq = 0
    for i in track_plan:
        track_msg = PoseStamped()
        track_msg.header.frame_id = 'map'
        track_msg.header.seq = seq
        track_msg.pose.position.x = i[0]
        track_msg.pose.position.y = i[1]
        path_msg.poses.append(track_msg)
        seq += 1
    path_pub.publish(path_msg)

    return goal


def pure_pursuit_control(state, tgoal):
    global ppk
    goal = math.atan2(
        tgoal['y'] -
        state.y,
        tgoal['x'] -
        state.x)  # (math.pi/4)*0.5
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
#    print('2goal:%.2f,curr:%.2f,diff:%.2f' %(goal,car,alpha))
    vcar = np.array([math.cos(car), math.sin(car)])
    vgoal = np.array([math.cos(tgoal['yaw']), math.sin(tgoal['yaw'])])
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

    alpha = math.atan2(vgoal[1], vgoal[0]) - math.atan2(vcar[1], vcar[0])
    if alpha > math.pi:
        alpha = alpha - 2 * math.pi
    if alpha < -math.pi:
        alpha = alpha + 2 * math.pi
    direc = direction_judge(state, tgoal)
    if direc > 0:
        alpha = alpha
    else:
        alpha = -alpha
#    print('4goal:%.2f,curr:%.2f,diff:%.2f' % (tgoal['yaw'], car, alpha))
    L = 1.06
    k = ppk # * 2
#    print('K=%.2f' %(k))
    Lf = k * 0.1
    delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)
#   print('alpha=%.2f,delta=%.2f' %(alpha,delta))
# print('goal:%.2f,curr:%.2f,diff:%.2f' %(math.atan2(cy - state.y, cx -
# state.x),state.yaw,alpha))
    return delta


def direction_judge(state, goal):
    x2 = math.pow(goal['x'] - state.x, 2)
    y2 = math.pow(goal['y'] - state.y, 2)
    s2 = math.sqrt(x2 + y2)
    alpha = math.atan2(goal['y'] - state.y, goal['x'] - state.x)
    car = np.array([math.cos(state.yaw), math.sin(state.yaw)])
    goal = np.array([math.cos(alpha), math.sin(alpha)])
    lc = np.sqrt(car.dot(car))
    lg = np.sqrt(goal.dot(goal))
    dcos = car.dot(goal) / (lc * lg)
    dyaw = np.arccos(dcos)
    if dyaw > math.pi / 2 or dyaw < -math.pi / 2:
        direction = -1
    else:
        direction = 1
#  print('sp%d,dyaw=%.2f' %(direction, dyaw))
    return direction


def pure_pursuit_callback(event):
    global state
    global car_speed
    global run
    global track_length

    if pose_recv and goal_recv:
#        print('track_length=%.2f' %(track_length))
        goal = calc_target_index(state, plan, track_length)
#        print('C=x:%.2f,y:%.2f,G=x:%.2f,y:%.2f' %(state.x,state.y,goal['x'],goal['y']))

        vx = abs(car_speed) * direction_judge(state, goal)
#        vx = 0.1 * direction_judge(state, goal)
        vth = pure_pursuit_control(state, goal)
        print('vx:%.2f,vth:%.2f' %(vx, vth))
        if vx == 0:
            vth = 0
        cmd_msg = Twist()
        run = True  # False
        if run:
            #      print('RUN')
            cmd_msg.linear.x = vx
            cmd_msg.angular.z = vth
        else:
            #      print('STOP')
            cmd_msg.linear.x = 0
            cmd_msg.angular.z = 0

        pos_msg = PointStamped()
        pos_msg.header.frame_id = 'map'
        pos_msg.point.x = goal['x']
        pos_msg.point.y = goal['y']

        cmd_pub.publish(cmd_msg)
        pos_pub.publish(pos_msg)


def cfg_callback(config, level):
    global track_length
    global ppk
#  global kd
    track_length = config.p
    ppk = config.i
    print('len:%.2f,k:%.2f' % (track_length, ppk))
#  kd = config.d
    return config


if __name__ == '__main__':
    try:
        rospy.init_node('pure_pursuit', anonymous=True)

        rospy.Subscriber('/mb_cmd', Twist, twist_callback)
        rospy.Subscriber('/luo_pos', Odometry, car_state_callback)
        rospy.Subscriber(
            '/move_base/TebLocalPlannerROS/local_plan',
            Path,
            path_callback)
        rospy.Subscriber('/car_goal_run', Twist, switch_callback)
        cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        pos_pub = rospy.Publisher('/track_pos', PointStamped, queue_size=1)
        path_pub = rospy.Publisher('/track_path', Path, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), pure_pursuit_callback)

#        srv = Server(PidConfig, cfg_callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
