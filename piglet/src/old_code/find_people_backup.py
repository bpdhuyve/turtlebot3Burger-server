#!/usr/bin/python
import rospy
import numpy as np
import actionlib
import math_functions as mf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Float32MultiArray, Bool
from actionlib_msgs.msg import GoalID
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

resolution = 0.0
origin_x  = 0.0
origin_y = 0.0

odom_initialized = False
grid_initialized = False
driving = False
exploring = True
backing_off = False

distance = 0.8
next_angle = 0.0
back_off_speed = 0.06
back_off_distance = 0.1

current_vel_x = 0.0
current_pos_x = 0.0
current_pos_y = 0.0
start_pos_x = 0.0
start_pos_y = 0.0

time_to_reach_goal = 10.0
goal_time_started = 0.0

#faces = [[230, 230], [230, 170], [170, 230], [170, 170]]
faces = []

face_index = 0

def get_faces(face_message):
	global faces
	global exploring
	face_message_array = face_message.data
	face_message_array = list(face_message_array)
	faces_new = []
	for i in range(0, len(face_message_array), 4):
		faces_new.append(face_message_array[i:i+4])
	if len(faces_new) > 0:
		print('got faces fom camera')
		exploring = False
		faces = faces_new
	else:
		exploring = True
	print(faces)

def getCurrentOdometry(odom):
	global current_vel_x
	global start_pos_x
	global start_pos_y
	global current_pos_x
	global current_pos_y
	global odom_initialized
	current_vel_x = odom.twist.twist.linear.x
	current_pos_x = odom.pose.pose.position.x
	current_pos_y = odom.pose.pose.position.y
	if not odom_initialized:
		start_pos_x = current_pos_x
		start_pos_y = current_pos_y
		odom_initialized = True


def getGridInfo(grid):
	global resolution
	global origin_x
	global origin_y
	global grid_initialized
	if(grid_initialized == False):
		origin_x = grid.info.origin.position.x
		origin_y = grid.info.origin.position.y
		resolution = grid.info.resolution
		grid_initialized = True

def edge_detected(sensor_value, args):
	global backing_off
	twist_pub = args[0]
	direction = args[1]
	if(sensor_value.data == True):
		if not backing_off:
			backing_off = True
			print('edge detected')
			client.cancel_all_goals()
			back_off(twist_pub, direction)

def face_found():
	global face_index
	global exploring
	#start = rospy.Time.now()
	#while(rospy.Time.now() - start < rospy.Duration.from_sec(2.0)):
	#	print('dirk')
	if exploring: return

	print('face found')
	client.cancel_all_goals()
	face_index += 1
	if(face_index == len(faces)):
		face_index = 0

def back_off(pub, direction):
	global current_pos_x
	global current_pos_y
	global backing_off
	global face_index
	current_face_index = face_index
	twist = Twist()
	twist.linear.x = direction * back_off_speed
	twist.linear.y = 0
	twist.linear.z = 0
	twist.angular.x = 0
	twist.angular.y = 0
	twist.angular.z = 0
	x0 = current_pos_x
	y0 = current_pos_y
	while(backing_off):
		pub.publish(twist)
		distance_squared = np.power(current_pos_x - x0, 2) + np.power(current_pos_y - y0, 2)
		if distance_squared > back_off_distance*back_off_distance:
			backing_off = False
			if not exploring:
				faces[current_face_index][1] = current_pos_x
				faces[current_face_index][2] = current_pos_y
				face_index -= 1
			print('backoff completed')
	face_found()
	goal = get_next_goal()
	move_navigation(goal)



def move_navigation(goal):
	global driving
	global face_index
	global goal_time_started
	goal_time_started = rospy.Time.now()
	driving = True
	client.wait_for_server()
	client.send_goal(goal)
	wait = client.wait_for_result(rospy.Duration.from_sec(20.0))
	if not wait:
		print('Periodically going to new face')
		face_found()
	else:
		face_found()
		return client.get_result()

def get_explore_goal():
	print('Choosing next exploration goal')
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

def get_face_goal():
	print('Face number: ' + str(face_index))
	print('Going to next face')
	face_coords = faces[face_index]
	face_x = face_coords[1]
	face_y = face_coords[2]
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = face_x
	goal.target_pose.pose.position.y = face_y
	goal.target_pose.pose.orientation.w = 1.0
	return goal


def get_next_goal():
	if(exploring):
		return get_explore_goal()
	else:
		return get_face_goal()

if __name__ == '__main__':
	global driving
	rospy.init_node('explorer')
	rospy.Rate(100)

	odom_sub = rospy.Subscriber('/odom', Odometry, getCurrentOdometry)
	grid_sub = rospy.Subscriber('/map', OccupancyGrid, getGridInfo)
	
	twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 5)

	sub_right = rospy.Subscriber('/custom/cliff_sensor_right', Bool, edge_detected, (twist_pub, 1))
	sub_left = rospy.Subscriber('/custom/cliff_sensor_left', Bool, edge_detected, (twist_pub, 1))
	sub_front_right = rospy.Subscriber('/custom/cliff_sensor_front_right', Bool, edge_detected, (twist_pub, -1))
	sub_front_left = rospy.Subscriber('/custom/cliff_sensor_front_left', Bool, edge_detected, (twist_pub, -1))

	face_pub = rospy.Subscriber('/custom/people_simple', Float32MultiArray, get_faces)

	while not rospy.is_shutdown():
		if odom_initialized and grid_initialized:
			try:
				if(driving == False):
					goal = get_next_goal()
					move_navigation(goal)
			except rospy.ROSInterruptException:
				rospy.loginfo("Navigation finished.")



	twist_stop = Twist()
	twist_stop.linear.x = 0
	twist_stop.linear.y = 0
	twist_stop.linear.z = 0
	twist_stop.angular.x = 0
	twist_stop.angular.y = 0
	twist_stop.angular.z = 0
	twist_pub.publish(twist_stop)	