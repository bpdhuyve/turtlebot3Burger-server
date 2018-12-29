#!/usr/bin/python
import rospy
import roslaunch
import numpy as np
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool
from actionlib_msgs.msg import GoalID
import random

driving = False
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)


def edge_detected(sensor_value):
	global driving
	#goalid = GoalID()
	#goalid.stamp = rospy.Time.now()
	#goalid.id = 'move_base'
	client.cancel_all_goals()
	driveToCenter()


def move(goal):
	global driving
	driving = True
	client.wait_for_server()
	client.send_goal(goal)
	wait = client.wait_for_result()
	if not wait:
		rospy.logerr("Action server not available")
		rospy.signal_shutdown("Action server not available!")
	else:
		driving = False
		return client.get_result()

def driveToCenter():
	print('driving to center')
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = 1.0
	goal.target_pose.pose.position.y = -1.5
	goal.target_pose.pose.orientation.w = 1.0
	move(goal)

def chooseRandomGoal():
	print('choosing random goal')
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = random.uniform(-2.0, 2.0)
	goal.target_pose.pose.position.y = random.uniform(-2.0, 2.0)
	goal.target_pose.pose.orientation.w = 1.0
	return goal

if __name__ == '__main__':
	global driving
	rospy.init_node('move_stop')
	sensor_sub = rospy.Subscriber('sensor_value', Bool, edge_detected)


	while not rospy.is_shutdown():

		try:
			if(driving == False):
				goal = chooseRandomGoal()
				move(goal)
		except rospy.ROSInterruptException:
			rospy.loginfo("Navigation test finished.")

	



