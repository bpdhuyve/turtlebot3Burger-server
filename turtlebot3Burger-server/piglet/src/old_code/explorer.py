#!/usr/bin/python
import rospy
import roslaunch
import numpy as np
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool
from actionlib_msgs.msg import GoalID
from nav_msgs.msg import Odometry
import random
from geometry_msgs.msg import Twist

exploring = True
driving = False
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
start_pos_x = None
start_pos_y = None
initialized = False
unregistered = False
distance = 3.0
next_angle = 0.0
current_vel_x = 0.0
back_off_speed = 0.22

def getInitialPose(odom):
	global initialized
	global start_pos_x
	global start_pos_y
	start_pos_x = odom.pose.pose.position.x
	start_pos_y = odom.pose.pose.position.y
	initialized = True

def getCurrentVelocity(odom):
	global current_vel_x
	current_vel_x = odom.twist.twist.linear.x

def edge_detected(sensor_value, args):
	twist_pub = args[0]
	direction = args[1]
	facereached_pub = args[2]
	facereached_pub.publish(True)
	if(start_pos_x != None and start_pos_y != None):
		if(sensor_value.data == True):
			#print('edge detected')
			client.cancel_all_goals()
			back_off(twist_pub, direction)

def back_off(pub, direction):
	twist = Twist()
	twist.linear.x = direction * back_off_speed
	twist.linear.y = 0
	twist.linear.z = 0
	twist.angular.x = 0
	twist.angular.y = 0
	twist.angular.z = 0
	pub.publish(twist)

def move_navigation(goal):
	global driving
	driving = True
	client.wait_for_server()
	client.send_goal(goal)
	wait = client.wait_for_result()
	backing_off = False
	if not wait:
		rospy.logerr("Action server not available")
		rospy.signal_shutdown("Action server not available!")
	else:
		driving = False
		return client.get_result()

def driveToCenter():
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = start_pos_x
	goal.target_pose.pose.position.y = start_pos_y
	goal.target_pose.pose.orientation.w = 1.0
	move_navigation(goal)

def chooseSmartGoal():
	print('choosing smart goal')
	global distance
	global next_angle
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = np.cos(next_angle)*distance + start_pos_x
	goal.target_pose.pose.position.y = np.sin(next_angle)*distance + start_pos_y
	goal.target_pose.pose.orientation.w = 1.0
	next_angle += 0.5
	return goal

def chooseRandomGoal():
	print('choosing random goal')
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = random.uniform(-2.0, 2.0)
	goal.target_pose.pose.position.y = random.uniform(-2.0, 2.0)
	goal.target_pose.pose.orientation.w = 1.0
	return goal

def move_to_face(face_goal):
	if(driving == False):
		move_navigation(face_goal)

if __name__ == '__main__':
	global driving
	rospy.init_node('move_stop')
	rospy.Rate(100)

	odom_sub = rospy.Subscriber('odom', Odometry, getInitialPose)
	vel_sub = rospy.Subscriber('odom', Odometry, getCurrentVelocity)
	facegoal_sub = rospy.Subscriber('/face_goals', MoveBaseGoal, move_to_face)
	facereached_pub = rospy.Publisher('/facereached', Bool, queue_size = 5)

	twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 5)

	sub_right = rospy.Subscriber('/custom/cliff_sensor_right', Bool, edge_detected, (twist_pub, -1, facereached_pub))
	sub_left = rospy.Subscriber('/custom/cliff_sensor_left', Bool, edge_detected, (twist_pub, -1, facereached_pub))
	sub_front_right = rospy.Subscriber('/custom/cliff_sensor_front_right', Bool, edge_detected, (twist_pub, -1, facereached_pub))
	sub_front_left = rospy.Subscriber('/custom/cliff_sensor_front_left', Bool, edge_detected, (twist_pub, -1, facereached_pub))
	sub_center_front = rospy.Subscriber('/custom/cliff_sensor_center_front', Bool, edge_detected, (twist_pub, -1, facereached_pub))
	sub_center_back = rospy.Subscriber('/custom/cliff_sensor_center_back', Bool, edge_detected, (twist_pub, 1, facereached_pub))


	while not rospy.is_shutdown():
		if (initialized != False):
			if (unregistered == False):
				odom_sub.unregister()
				unregistered = True
			if (exploring):
				try:
					if(driving == False and start_pos_x != None):
						goal = chooseSmartGoal()
						move_navigation(goal)
				except rospy.ROSInterruptException:
					rospy.loginfo("Navigation test finished.")

	twist_stop = Twist()
	twist_stop.linear.x = -0.05
	twist_stop.linear.y = 0
	twist_stop.linear.z = 0
	twist_stop.angular.x = 0
	twist_stop.angular.y = 0
	twist_stop.angular.z = 0
	twist_pub.publish(twist_stop)