#!/usr/bin/python
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import tf
import roslaunch
import numpy as np
import math_functions as mf

edge_range = 0
old_map = []
old_grid = OccupancyGrid()
changed_map = []
changed_grid = OccupancyGrid()
pos_x = None
pos_y = None
rotation = None
edge_coordinates = []

sensor_right_rotation = -3.14 + 0.5
sensor_left_rotation = 3.14 - 0.5
sensor_front_right_rotation = 0.5
sensor_front_left_rotation = -0.5
#cliff_sensor_center_front_rotation = 0
#cliff_sensor_center_back_rotation = 3.14

cliff_sensor_right_position = None
cliff_sensor_left_position = None
cliff_sensor_front_right_position = None
cliff_sensor_front_left_position = None
#cliff_sensor_center_front_position = None
#cliff_sensor_center_back_position = None

sensor_distance = 0.3
resolution = None
origin_x = None
origin_y = None

def getPose(odom):
	global pos_x
	global pos_y
	global rotation
	pos_x = odom.pose.pose.position.x
	pos_y = odom.pose.pose.position.y
	(roll, pitch, rotation) = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])

def saveMap(grid):
	global old_map
	global old_grid
	global resolution
	global origin_x
	global origin_y
	old_map = grid.data
	old_grid = grid
	resolution = grid.info.resolution
	origin_x = grid.info.origin.position.x
	origin_y = grid.info.origin.position.y

def setSensorPositions():
	global cliff_sensor_right_position
	global cliff_sensor_left_position
	global cliff_sensor_front_right_position
	global cliff_sensor_front_left_position
	#global cliff_sensor_center_front_position
	#global cliff_sensor_center_back_position

	cliff_sensor_right_position = mf.getSensorPosition(rotation, sensor_right_rotation, sensor_distance, pos_x, pos_y, origin_x, origin_y, resolution)
	cliff_sensor_left_position = mf.getSensorPosition(rotation, sensor_left_rotation, sensor_distance, pos_x, pos_y, origin_x, origin_y, resolution)
	cliff_sensor_front_right_position = mf.getSensorPosition(rotation, sensor_front_right_rotation, sensor_distance, pos_x, pos_y, origin_x, origin_y, resolution)
	cliff_sensor_front_left_position = mf.getSensorPosition(rotation, sensor_front_left_rotation, sensor_distance, pos_x, pos_y, origin_x, origin_y, resolution)
	#cliff_sensor_center_front_position = mf.getSensorPosition(rotation, cliff_sensor_center_front_rotation, sensor_distance, pos_x, pos_y, origin_x, origin_y, resolution)
	#cliff_sensor_center_back_position = mf.getSensorPosition(rotation, cliff_sensor_center_back_rotation, sensor_distance, pos_x, pos_y, origin_x, origin_y, resolution)

def changeMap(map):
	global changed_map
	global changed_grid
	changed_map=list(map)
	map_width = old_grid.info.width
	map_height = old_grid.info.height
	changed_map = np.reshape(changed_map, (map_height, map_width))
	for coord in edge_coordinates:
		x = coord[0]
		y = coord[1]
		edge_points = np.ones((edge_range*2 + 1, edge_range*2 + 1))*100
		changed_map[y-edge_range : y+edge_range+1, x-edge_range : x+edge_range+1] = edge_points
		
	changed_map = tuple(changed_map.flatten())
	changed_grid.header = old_grid.header
	changed_grid.info = old_grid.info
	changed_grid.data = changed_map

def setEdgeSensorRight(sensor_value):
	if(cliff_sensor_right_position != None and sensor_value.data == True and cliff_sensor_right_position not in edge_coordinates):
		edge_coordinates.append(cliff_sensor_right_position)

def setEdgeSensorLeft(sensor_value):
	if(cliff_sensor_left_position != None and sensor_value.data == True and cliff_sensor_left_position not in edge_coordinates):
		edge_coordinates.append(cliff_sensor_left_position)

def setEdgeSensorFrontRight(sensor_value):
	if(cliff_sensor_front_right_position != None and sensor_value.data == True and cliff_sensor_front_right_position not in edge_coordinates):
		edge_coordinates.append(cliff_sensor_front_right_position)

def setEdgeSensorFrontLeft(sensor_value):
	if(cliff_sensor_front_left_position != None and sensor_value.data == True and cliff_sensor_front_left_position not in edge_coordinates):
		edge_coordinates.append(cliff_sensor_front_left_position)

#def setEdgeSensorCenterFront(sensor_value):
#	if(cliff_sensor_center_front_position != None and sensor_value.data == True and cliff_sensor_center_front_position not in edge_coordinates):
#		edge_coordinates.append(cliff_sensor_center_front_position)
#
#def setEdgeSensorCenterBack(sensor_value):
#	if(cliff_sensor_center_back_position != None and sensor_value.data == True and cliff_sensor_center_back_position not in edge_coordinates):
#		edge_coordinates.append(cliff_sensor_center_back_position)


if __name__ == '__main__':
	rospy.init_node('mapChanger', anonymous=True)
	rate = rospy.Rate(200)
	pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
	sub_right = rospy.Subscriber('/custom/cliff_sensor_right', Bool, setEdgeSensorRight)
	sub_left = rospy.Subscriber('/custom/cliff_sensor_left', Bool, setEdgeSensorLeft)
	sub_front_right = rospy.Subscriber('/custom/cliff_sensor_front_right', Bool, setEdgeSensorFrontRight)
	sub_front_left = rospy.Subscriber('/custom/cliff_sensor_front_left', Bool, setEdgeSensorFrontLeft)
	#sub_center_front = rospy.Subscriber('/custom/cliff_sensor_center_front', Bool, setEdgeSensorCenterFront)
	#sub_center_back = rospy.Subscriber('/custom/cliff_sensor_center_back', Bool, setEdgeSensorCenterBack)

	map_sub = rospy.Subscriber('slammap', OccupancyGrid, saveMap)
	odom_sub = rospy.Subscriber('odom', Odometry, getPose)
	while not rospy.is_shutdown():
		#print(edge_coordinates)
		if pos_x != None and pos_y != None and len(old_map) > 0 and rotation != None:
			setSensorPositions()
			changeMap(old_map)
			if(len(changed_map) > 0):
				pub.publish(changed_grid)
