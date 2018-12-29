#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import roslaunch
import numpy as np
import actionlib

def movebase_client():
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	client.wait_for_server()

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = 3.0
	goal.target_pose.pose.orientation.w = 3.0

	client.send_goal(goal)
	wait = client.wait_for_result()
	if not wait:
		rospy.logerr("Action server not available")
		rospy.signal_shutdown("Action server not available!")
	else:
		return client.get_result()

if __name__ == '__main__':
	try:
		rospy.init_node('movebase_client_py')
		result = movebase_client()
		if result:
			rospy.loginfo("Goal execution done!")
	except rospy.ROSInterruptException:
		rospy.loginfo("Navigation test finished.")