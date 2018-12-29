#!/usr/bin/python
import rospy
import numpy as np
from nav_msgs.msg import Odometry, OccupancyGrid
import math

faces = [[0.0, 0.0, 1], [0.0, 0.0, 2], [0.0, 0.0, 3], [0.0, 0.0, 4]]
processed_faces = None

table_map = None
odom_initialized = False

resolution = None
origin_x = None
origin_y = None

current_pos_x = 0.0
current_pos_y = 0.0

def getCurrentOdometry(odom):
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
		for face in faces:
			face[0] = face[0] + np.random.rand()*3
			face[1] = face[1] + np.random.rand()*3
		odom_initialized = True

def getMap(grid):
	global resolution
	global origin_x
	global origin_y
	global table_map
	resolution = grid.info.resolution
	origin_x = grid.info.origin.position.x
	origin_y = grid.info.origin.position.y

	map_width = grid.info.width
	map_height = grid.info.height

	grid_map = grid.data
	grid_map = list(grid_map)
	grid_map = np.reshape(table_map, (map_height, map_width))

	table_map = grid_map

def getFirstCollision(face):
	len_line = math.sqrt(math.pow(current_pos_x - face[0], 2), math.pow(current_pos_y - face[1], 2))
	len_step = len_line * resolution
	num_steps = int(len_line / len_step)
	distance_x = current_pos_x - face[0]
	distance_y = current_pos_y - face[1]
	len_step_x = distance_x * resolution
	len_step_y = distance_y * resolution

	x_step = 0
	y_step = 0

	for i in range(0, num_steps):

		distance_x = current_pos_x - face[0]
		distance_y = current_pos_y - face[1]
		direction_x = copysign(1, distance_x)
		direction_y = copysign(1, distance_y)
		x_step = len_step	


def process_faces():
	for face in faces:
		collision_coord = get_first_collision(face)
		



if __name__ == '__main__':
	map_sub = rospy.Subscriber('slammap', OccupancyGrid, getMap)
	odom_sub = rospy.Subscriber('odom', Odometry, getCurrentOdometry)

	while not rospy.is_shutdown():
		process_faces()
