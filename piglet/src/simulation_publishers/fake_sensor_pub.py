#!/usr/bin/python
import std_msgs.msg
from nav_msgs.msg import Odometry, OccupancyGrid
import rospy
import numpy as np
import tf
import math_functions as mf

sensor_right_rotation = -3.14 + 0.5
sensor_left_rotation = 3.14 - 0.5
sensor_front_right_rotation = 0.5
sensor_front_left_rotation = -0.5
#sensor_center_front_rotation = 0
#sensor_center_back_rotation = 3.14

sensor_distance = 0.2
origin_x = 0
origin_y = 0
resolution = 1

sensor_right_position = None
sensor_left_position = None
sensor_front_right_position = None
sensor_front_left_position = None
#sensor_center_front_position = None
#sensor_center_back_position = None

def getOrigin(grid):
	global resolution
	global origin_x
	global origin_y
	origin_x = grid.info.origin.position.x
	origin_y = grid.info.origin.position.y
	resolution = grid.info.resolution

def getSensorLocations(odom):
	global sensor_right_position
	global sensor_left_position
	global sensor_front_right_position
	global sensor_front_left_position
	#global sensor_center_front_position
	#global sensor_center_back_position

	pos_x = odom.pose.pose.position.x
	pos_y = odom.pose.pose.position.y
	(roll, pitch, rotation) = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])

	sensor_right_position = mf.getSensorPosition(rotation, sensor_right_rotation, sensor_distance, pos_x, pos_y, origin_x, origin_y, resolution)
	sensor_left_position = mf.getSensorPosition(rotation, sensor_left_rotation, sensor_distance, pos_x, pos_y, origin_x, origin_y, resolution)
	sensor_front_right_position = mf.getSensorPosition(rotation, sensor_front_right_rotation, sensor_distance, pos_x, pos_y, origin_x, origin_y, resolution)
	sensor_front_left_position = mf.getSensorPosition(rotation, sensor_front_left_rotation, sensor_distance, pos_x, pos_y, origin_x, origin_y, resolution)
	#sensor_center_front_position = mf.getSensorPosition(rotation, sensor_center_front_rotation, sensor_distance, pos_x, pos_y, origin_x, origin_y, resolution)
	#sensor_center_back_position = mf.getSensorPosition(rotation, sensor_center_back_rotation, sensor_distance, pos_x, pos_y, origin_x, origin_y, resolution)

	print("own position: ")
	print(pos_x)
	print(pos_y)
	print("Sensor position: ")
	print(sensor_right_position)
	
def outOfBounds(coords):
	offset_x = 200
	offset_y = 160
	square_height = 50
	square_width = 50
	x, y = coords
	if x > square_width + offset_x or x < 0 + offset_x or y > square_height + offset_y or y < 0 + offset_y:
		return True
	else: return False

if __name__ == '__main__':
	rospy.init_node('fake_sensor_pub')

	odom_sub = rospy.Subscriber('/odom', Odometry, getSensorLocations)
	map_sub = rospy.Subscriber('/slammap', OccupancyGrid, getOrigin)

	rospy.Rate(200)

	pub_right = rospy.Publisher(
		'/custom/cliff_sensor_right',
		std_msgs.msg.Bool,
		queue_size=1
	)
	pub_left = rospy.Publisher(
		'/custom/cliff_sensor_left',
		std_msgs.msg.Bool,
		queue_size=1
	)
   	pub_front_right = rospy.Publisher(
		'/custom/cliff_sensor_front_right',
		std_msgs.msg.Bool,
		queue_size=1
	)
   	pub_front_left = rospy.Publisher(
		'/custom/cliff_sensor_front_left',
		std_msgs.msg.Bool,
		queue_size=1
	)
   	#pub_center_front = rospy.Publisher(
	#	'/custom/cliff_sensor_center_front',
	#	std_msgs.msg.Bool,
	#	queue_size=1
	#)
   	#pub_center_back = rospy.Publisher(
	#	'/custom/cliff_sensor_center_back',
	#	std_msgs.msg.Bool,
	#	queue_size=1
	#)

	while not rospy.is_shutdown():
		if sensor_right_position != None and sensor_left_position != None and sensor_front_right_position != None and sensor_front_left_position:# != None and sensor_center_front_position != None and sensor_center_back_position != None:
			#print(sensor_right_position)
			#print(origin_y)
			pub_right.publish(outOfBounds(sensor_right_position))
			pub_left.publish(outOfBounds(sensor_left_position))
			pub_front_right.publish(outOfBounds(sensor_front_right_position))
			pub_front_left.publish(outOfBounds(sensor_front_left_position))
			#pub_center_front.publish(outOfBounds(sensor_center_front_position))
			#pub_center_back.publish(outOfBounds(sensor_center_back_position))
