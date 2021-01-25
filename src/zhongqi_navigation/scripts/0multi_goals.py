#!/usr/bin/env python
import rospy
import numpy
import tf
from smach import StateMachine
from smach_ros import SimpleActionState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
import threading
from multiprocessing.pool import ThreadPool
waypoints = [
    ['P1', (1.25, 1, 145)],
    ['P2', (1, 1.25, 145)],
    ['P3', (1, 1.75, 45)],
    ['P4', (1.25, 2, 45)],
    ['P5', (1.75, 2, -45)],
    ['P6', (2, 1.75, -45)],
    ['P7', (2, 1.25, -145)],
    ['P8', (1.75, 1, -145)],
]

if __name__ == '__main__':
    try:
        rospy.init_node('multi_goals')
        patrol = StateMachine(['succeeded', 'aborted', 'preempted'])
        with patrol:
            for i, w in enumerate(waypoints):
#                print('i=%d,w=%.2f' %(i,w))
                print('i:%d, w:%s l:%d' %(i,w,len(waypoints)))
                goal_pose = MoveBaseGoal()
                goal_pose.target_pose.header.frame_id = 'map'
                goal_pose.target_pose.pose.position.x = w[1][0]
                goal_pose.target_pose.pose.position.y = w[1][1]
                goal_pose.target_pose.pose.position.z = 0.0

                goal_radian = numpy.radians(w[1][2])
                print('y:%d' %(waypoints[(i + 1)]))
                q = tf.transformations.quaternion_from_euler(0, 0, goal_radian)
                q_msg = Quaternion(*q)

                goal_pose.target_pose.pose.orientation = q_msg

                StateMachine.add(w[0],
                                  SimpleActionState(
                     'move_base', MoveBaseAction, goal=goal_pose),
                     transitions={
                     'succeeded': waypoints[(i + 1) % len(waypoints)][0]}
                 )
#        patrol.execute()
        pool = ThreadPool(processes=1)
        async_result = pool.apply_async(patrol.execute())
    except rospy.ROSInterruptException: 
        pass
    
