#!/usr/bin/python
import sys
import rospy 
import math
import time
import signal
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

class MoveBase(object): 
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def move_client(self,goal): 
        self.client.send_goal(goal)
        wait = self.client.wait_for_server() 
        print("here")     
        if not wait:
            print("not wait")  
            rospy.logerr("Action server not available!") 
            rospy.signal_shutdown("Action server not available!")   
        else: 
            print("returned")
            return self.client.get_result()


if __name__ == '__main__': 
    rospy.init_node('move_controller') 
    movebase = MoveBase()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map" 
    goal.target_pose.header.stamp = rospy.Time.now() 
    goal.target_pose.pose.position.x = 0.4
    goal.target_pose.pose.orientation.w = 1.0
    try:
        result = movebase.move_client(goal)        
    except rospy.ROSInterruptException: 
        rospy.loginfo("Navigation test finished.")
