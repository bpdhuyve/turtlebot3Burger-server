#!/usr/bin/python
import rospy
import numpy as np
import actionlib
import math_functions as mf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Float32MultiArray, Bool, Int32
from actionlib_msgs.msg import GoalID
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

odom_initialized = False
driving = False
exploring = True
backing_off = False
back_off_started = False

explore_distance = 0.8
angle_increment = 1.0

next_angle = 0.0
back_off_speed = 0.06
back_off_distance = 0.1

current_pos_x = 0.0
current_pos_y = 0.0
start_pos_x = 0.0
start_pos_y = 0.0

exploration_time_to_reach = 10.0
face_time_to_reach = 20.0

mnm_allowed = False #While this is true, we check wether or not mnm is taken
mnm_taken = False

faces = [] #List of ids and coordinates of people, detected by face recognition
heathens = [] #List of ids of people who don't like mnms

face_index = 0 #Index of the face where piglet is going

# LIST of foice commands: [hello: 0, bye: 1, take_an_mnm: 2, welcome: 3, No mnmns: 4, Enjoy: 5]


rospy.init_node('explorer')
rospy.Rate(100)

twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1, latch=True) # The publisher which is used to send movements untied to navigation stack
messageVoicePub = rospy.Publisher("/custom/voice_message", Int32, queue_size =1)

def set_exploring():
	global exploring
	exploring = True
	publish_voice(6)

def set_serving():
	global exploring
	exploring = False
	publish_voice(7)


def publish_voice(x):
	message = Int32()
	message.data = x
	messageVoicePub.publish(message)

# Callback if mnmsensor is triggered (false means mnm is taken)
def mnm_taken(sensor_value):
	global heathens
	global mnm_taken
	if mnm_allowed:
		if(sensor_value.data == False):
			mnm_taken = True

# Callback that checks if new faces are added, or if faces are removed
def get_faces(face_message):
	global faces
	global exploring
	face_message_array = face_message.data
	if len(face_message_array) == 0 and exploring == False:
		set_exploring()
		return
	face_message_array = list(face_message_array)
	new_face_ids = []
	face_ids = [face[0] for face in faces]



	for i in range(0, len(face_message_array), 4): #append new faces
		new_face = face_message_array[i:i+4]
		new_face_id = new_face[0]
		new_face_ids.append(new_face_id)
		if new_face_id not in face_ids:
			faces.append(new_face)
			publish_voice(3)
			print('new face found: '+str(new_face))

	index = 0
	while index < len(face_ids):
		
		if face_ids[index] not in new_face_ids:
			print('face has been removed')
			del face_ids[index]
			del faces[index]
		else: index += 1

# Callback to get start position of the robot, necessary for exploring
def getCurrentOdometry(odom):
	global start_pos_x
	global start_pos_y
	global current_pos_x
	global current_pos_y
	global odom_initialized
	current_pos_x = odom.pose.pose.position.x
	current_pos_y = odom.pose.pose.position.y
	if not odom_initialized:
		print('odom initizalized')
		start_pos_x = current_pos_x
		start_pos_y = current_pos_y
		odom_initialized = True

# Callback when the sensor detect that the robot is driving of the edge
def edge_detected(sensor_value, direction):
	global backing_off
	if(sensor_value.data == True):
		if not backing_off:
			backing_off = True
			print('edge detected')
			client.cancel_all_goals()
			back_off(direction)

# Function that is called when we assume a face is found.
# Robot waits a bit and checks if an mnm is taken.
def face_found():
	global face_index
	global heathens
	global mnm_allowed
	global mnm_taken
	publish_voice(2)
	start = rospy.Time.now()
	print('face found')
	client.cancel_all_goals()
	mnm_allowed = True
	mnm_taken = False
	rate = rospy.Rate(5)
	while(rospy.Time.now() - start < rospy.Duration.from_sec(8.0)): #Wait a bit to allow person to take mnm
		twist = Twist()
		twist.linear.x = 0
		twist.linear.y = 0
		twist.linear.z = 0
		twist.angular.x = 0
		twist.angular.y = 0
		twist.angular.z = 0
		twist_pub.publish(twist)
		rate.sleep()
	mnm_allowed = False
	if(mnm_taken == False):
		publish_voice(4)
		print('HEATHEN DETECTED')
		if(faces[face_index][0] not in heathens):
			heathens.append(faces[face_index][0])

	else:
		publish_voice(5)
	mnm_taken = False
	iteration_index = 0
	previous_index = face_index

	if face_index + 1 == len(faces):
		set_exploring()

	face_index = (face_index + 1) % len(faces)
	while (faces[face_index % len(faces)][0] in heathens):
		face_index = (face_index + 1) % len(faces)
		iteration_index += 1
		if(iteration_index > len(faces)):
			heathens = []

	if face_index == previous_index:
		set_exploring()

	print('next face: '+str(faces[face_index]))
	driving = False

# Function that is called when the robot is driving off the edge.
# The robot drives back a certain distance (direction is decided by which sensor was triggered)
def back_off(direction):
	global current_pos_x
	global current_pos_y
	global backing_off
	global face_index
	global back_off_started
	global twist_pub
	global explore_distance
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
	if back_off_started: return

	rate = rospy.Rate(20)
	while(backing_off):
		back_off_started = True
		twist_pub.publish(twist)
		distance_squared = np.power(current_pos_x - x0, 2) + np.power(current_pos_y - y0, 2)
		if distance_squared > back_off_distance*back_off_distance:
			backing_off = False
			if exploring:
				#explore_distance_squared = np.power(current_pos_x - x0, 2) + np.power(current_pos_y - y0, 2)
				#explore_distance = np.sqrt(explore_distance_squared)
				explore_distance -= 0.1
			if not exploring:
				faces[current_face_index][1] = current_pos_x
				faces[current_face_index][2] = current_pos_y
			print('backoff completed')
		rate.sleep()
	back_off_started = False

# Method that sends a goal to the navigation stack.
def move_navigation(goal):
	global driving
	global face_index
	driving = True
	client.wait_for_server()
	client.send_goal(goal)
	if exploring: time_to_reach = exploration_time_to_reach
	else: time_to_reach = face_time_to_reach
	wait = client.wait_for_result(rospy.Duration.from_sec(time_to_reach))
	driving = False
	if not wait:
		print('Periodically going to new goal')
		if not exploring: face_found()

	else:
		if not exploring: face_found()
		return client.get_result()

#Method that gets a new goal if the robot is in exploration modus
def get_explore_goal():
	print('Choosing next exploration goal')
	global explore_distance
	global next_angle
	global exploring
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = np.cos(next_angle)*explore_distance + start_pos_x
	goal.target_pose.pose.position.y = np.sin(next_angle)*explore_distance + start_pos_y
	goal.target_pose.pose.orientation.w = 1.0
	print(next_angle + angle_increment)
	if(next_angle + angle_increment > 6.28):
		if(len(faces) > 0):
			next_angle = 0
			print('Exploration phase finished')
			set_serving()
			return get_face_goal()
	next_angle = (next_angle + angle_increment) % 6.28
	return goal

#Method that gets a goal to the next face
def get_face_goal():
	print('Going to face number: ' + str(face_index))
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

# Methode that chooses a goal, based on wheter or not the robot is still exploring
def get_next_goal():
	if(exploring):
		return get_explore_goal()
	else:
		return get_face_goal()

if __name__ == '__main__':
	global driving

	odom_sub = rospy.Subscriber('/odom', Odometry, getCurrentOdometry)
	
	sub_right = rospy.Subscriber('/custom/cliff_sensor_right', Bool, edge_detected, 1, queue_size = 1)
	sub_left = rospy.Subscriber('/custom/cliff_sensor_left', Bool, edge_detected, 1, queue_size = 1)
	sub_front_right = rospy.Subscriber('/custom/cliff_sensor_front_right', Bool, edge_detected, -1, queue_size = 1)
	sub_front_left = rospy.Subscriber('/custom/cliff_sensor_front_left', Bool, edge_detected, -1, queue_size = 1)

	sub_mnm = rospy.Subscriber('/custom/cliff_sensor_center_front', Bool, mnm_taken, queue_size = 1)

	face_pub = rospy.Subscriber('/custom/people_simple', Float32MultiArray, get_faces)


	while not rospy.is_shutdown():
		if odom_initialized:
			try:
				if(driving == False):
					goal = get_next_goal()
					move_navigation(goal)
			except rospy.ROSInterruptException:
				rospy.loginfo("Navigation finished.")