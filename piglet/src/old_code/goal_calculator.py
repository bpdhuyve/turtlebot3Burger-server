#!/usr/bin/python
from nav_msgs.msg import Odometry, OccupancyGrid
from move_base_msgs.msg import MoveBaseGoal
from std_msgs.msg import Bool
import rospy
import numpy
import math_functions as mf

initialized = False
resolution = None
origin_x  = None
origin_y = None
current_face_index = 0

faces = [[230, 230], [230, 190], [190, 230], [190, 190]]

def initialize(grid):
	global resolution
	global origin_x
	global origin_y
	global initialized
	if(initialized == False):
		origin_x = grid.info.origin.position.x
		origin_y = grid.info.origin.position.y
		resolution = grid.info.resolution
		initialized = True

def calculate_goal(face_coords):
	face_x, face_y = mf.coordinates_to_position(face_coords, origin_x, origin_y, resolution)
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = face_x
	goal.target_pose.pose.position.y = face_y
	goal.target_pose.pose.orientation.w = 1.0
	return goal

def next_face(bool):
	global current_face_index
	if(bool.data == True):
		current_face_index += 1
		if(current_face_index == len(faces)):
			current_face_index = 0

if __name__ == '__main__':
	rospy.init_node('goal_calculator')

	map_sub = rospy.Subscriber('/slammap', OccupancyGrid, initialize)
	facereached_sub = rospy.Subscriber('/facereached', Bool, next_face)
	facegoal_pub = rospy.Publisher('/face_goals', MoveBaseGoal, queue_size = 5)


	while not rospy.is_shutdown():
		if(initialized):
			face_goal = calculate_goal(faces[current_face_index])
			facegoal_pub.publish(face_goal)
