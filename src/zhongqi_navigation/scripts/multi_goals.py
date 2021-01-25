#!/usr/bin/env python
import rospy
import numpy
import tf
import smach
import smach_ros
from smach import StateMachine
from smach_ros import SimpleActionState
from smach_ros import MonitorState, IntrospectionServer
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
import threading
from multiprocessing.pool import ThreadPool
waypoints = [
    [1,'P1', (-2, -1, -100)],
    [2,'P2', (0, 0.5, 180)], #0, 0.5, 180
    [3,'P3', (2, 0.5, 180)], #2, 0.5, 180
    [4,'P4', (0, 0.5, 180)]

]

'''
    [1,'P1', (0, 0.5, 180)],
    [2,'P2', (-2, -1.5, -90)],
    [3,'P3', (0, -3.5, 0)],
    [4,'P4', (2.5, -3.5, 0)],
    [5,'P5', (4.5, -1.5, 90)],
    [6,'P6', (2.5, 0.5, 180)]
1012
    [1,'P1', (15, 1.4, 90)],
    [2,'P2', (15, 2.5, 90)],
    [3,'P3', (15, 8, 100)],
    [4,'P4', (12.5, 10, 180)],
    [5,'P5', (11, 10, 180)],
    [6,'P6', (6.5, 10, 180)], 
    [7,'P7', (1, 10, 180)],
    [8,'P8', (-1, 8, -80)],
    [9,'P9', (-0.5, 6, -90)],
    [10,'P10', (-0.5, 2, -90)],
    [11,'P11', (1, -0.3, 0)],
    [12,'P12', (2, -0.3, 0)],
    [13,'P13', (7.5, -0.3, 0)],
    [14,'P14', (13, -0.3, 0)]

    [1,'P1', (14.3, 2.8, -90)],
    [2,'P2', (12.3, 4.8, 0)],
    [3,'P3', (13, 4.8, 0)],
    [4,'P4', (15, 6.8, 90)],
    [5,'P5', (15, 8, 90)],
    [6,'P6', (13, 10, 180)], 
    [7,'P7', (2.3, 10, 180)],
    [8,'P8', (4.3, 8, 90)],
    [9,'P9', (4.3, 7, 90)],
    [10,'P10', (6.3, 9, 0)],
    [11,'P11', (12.2, 9.2, 0)],
    [12,'P12', (14.2, 7.2, -90)]

    [1,'P1', (15, 1.4, 90)],
    [2,'P2', (15, 2.5, 90)],
    [3,'P3', (15, 8, 90)],
    [4,'P4', (13, 10, 180)],
    [5,'P5', (11, 10, 180)],
    [6,'P6', (6.5, 10, 180)], 
    [7,'P7', (1, 10, 180)],
    [8,'P8', (-1, 8, -90)],
    [9,'P9', (-0.5, 7, -90)],
    [10,'P10', (-1, 2, -90)],
    [11,'P11', (1, -0.3, 0)],
    [12,'P12', (2, -0.3, 0)],
    [13,'P13', (7.5, -0.3, 0)],
    [14,'P14', (13, -0.3, 0)]
'''


class check(smach.State):
  def  __init__(self):
    smach.State.__init__(self, outcomes=['run','stop','quit'])
    self.subscriber = rospy.Subscriber('/car_goal_run', Twist, self.cb)
    self.got = False
    
  def cb(self, msg):
      if msg.angular.x == 1:
#          print('run') 
          self.got = True
      else:
#          print('stop') 
          self.got = False
          
  def execute(self, userdata): 
#      print(userdata.check_in)
#      print('in-data:%s,type:%s' %(userdata.in_, type(userdata.in_)))
#      userdata.out_ = 'P'+str(int(userdata.in_.split('P',0)[1])+1)
#      a = 'P'+str(int(userdata.check_in.split('P')[1])+1)
 #     userdata.check_out = a
#      print('out-data:%s,type:%s' %(userdata.out_, type(userdata.out_)))
      if self.got: 
#          userdata.out = userdata.in + 1
          return 'run'
      else: 
          return 'stop'
          
          
class loop_check(smach.State):
  def  __init__(self):
    smach.State.__init__(self, outcomes=['run','stop','quit'])
    self.subscriber = rospy.Subscriber('/car_goal_run', Twist, self.cb)
    self.got = False
  def cb(self, msg):
      if msg.angular.x == 1: # and msg.angular.z == 1:
          self.got = True
      else:
          self.got = False
  def execute(self, userdata):
      if self.got: 
          return 'run'
      else: 
          return 'stop'


def togoalmsg(point):
#    print('point:%s' %(point))
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = point[2][0]
    goal_pose.target_pose.pose.position.y = point[2][1]
    goal_pose.target_pose.pose.position.z = 0.0
    goal_radian = numpy.radians(point[2][2])
    q = tf.transformations.quaternion_from_euler(0, 0, goal_radian)
    q_msg = Quaternion(*q)
    goal_pose.target_pose.pose.orientation = q_msg
    return goal_pose

class run(smach.State):
  def  __init__(self):
    smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])
#    print('runnnnnnn')
  def execute(self, userdata): 
    SimpleActionState('move_base',
                      MoveBaseAction,
                      goal=userdata.in_)
def patrol():
    print('patrol') 
    sm = smach.StateMachine(outcomes=['run','stop'])
    sm.userdata.sm_counter = 'P1'
    sm.userdata.sm_goto = ' '
    with sm:
        for i, w in enumerate(waypoints):
            print('i:%d, w:%s l:%d' %(i,w,len(waypoints)))   
            sm_goto = togoalmsg(w)       
            StateMachine.add(w[1],
#                             run(),
                             SimpleActionState('move_base',
                                               MoveBaseAction,
                                               goal=togoalmsg(w)),
                             transitions={'succeeded': 'CHECK'+str(w[0]),
                                          'preempted':'CHECK'+str(w[0]),
                                          'aborted':'CHECK'+str(w[0])})
                                          
            if i<len(waypoints)-1:
                StateMachine.add('CHECK'+str(w[0]),
                             check(), 
                             transitions={'run': waypoints[i+1][1],
                                          'stop': 'CHECK'+str(w[0]),
                                          'quit': 'stop'})
            else:               
                StateMachine.add('CHECK'+str(w[0]),
                             loop_check(), 
                             transitions={'run': waypoints[0][1],
                                          'stop': 'CHECK'+str(w[0]),
                                          'quit': 'stop'})     
    return sm
  
class cmd_recv(smach.State):
  def  __init__(self):
    smach.State.__init__(self, outcomes=['run','stop'])
    self.subscriber = rospy.Subscriber('/car_goal_run', Twist, self.cb)
    self.got = False
  def cb(self, msg):
      if msg.angular.x == 1:
#          print('run') 
          self.got = True
      else:
#          print('stop') 
          self.got = False
     
  def execute(self, data):
      if self.got:

          return 'run'
      else: 

          return 'stop'

           
def patrol1():
  sm = smach.StateMachine(outcomes=['run','stop'])
  print('stop') 
  with sm:
    smach.StateMachine.add('ROOM1', cmd_recv(),
                            transitions={'run': 'stop'})         
  return sm  


if __name__ == '__main__':
    rospy.init_node('multi_goals')
    try:                                    
        wait_cmd = StateMachine(['run','stop'])
#        wait_cmd = smach.Concurrence(outcomes=['run','stop'],
#                                     default_outcome='stop',)
        with wait_cmd:
            smach.StateMachine.add('STOP',
                                   cmd_recv(),
                                   transitions={'run':'RUN',
                                                'stop':'STOP'})
            smach.StateMachine.add('RUN',
                                   patrol(),
                                   transitions={'stop':'STOP'})  
                                    
#        outcome = wait_cmd.execute()
        pool = ThreadPool(processes=1)
        async_result = pool.apply_async(wait_cmd.execute)
        rospy.spin()

    except rospy.ROSInterruptException: 
        pass
        
