#!/usr/bin/python
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import tf
import roslaunch
import numpy as np
import math

old_map = []
old_grid = OccupancyGrid()
changed_map = []
changed_grid = OccupancyGrid()
pos_x = None
pos_y = None
rotation = None
edge_coordinates = []
sensor1 = False
sensor2 = False
sensor3 = False
sensor4 = False
sensor1_rotation = 0.35
sensor2_rotation = -0.35
sensor3_rotation = 0.9
sensor4_rotation = 3.14
sensor1_position = None
sensor2_position = None
sensor3_position = None
sensor4_position = None
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

def convertAngle(rotation):
	if rotation > 0: return rotation
	else: return 6.283 + rotation

def setSensorPositions():
	global sensor1_position
	global sensor2_position
	global sensor3_position
	global sensor4_position

	sensor1_pos_x = np.cos(rotation + sensor1_rotation)*sensor_distance + pos_x
	sensor1_pos_y = np.sin(rotation + sensor1_rotation)*sensor_distance + pos_y
	sensor1_position = position_to_coordinates(sensor1_pos_x, sensor1_pos_y)
	sensor2_pos_x = np.cos(rotation + sensor2_rotation)*sensor_distance + pos_x
	sensor2_pos_y = np.sin(rotation + sensor2_rotation)*sensor_distance + pos_y
	sensor2_position = position_to_coordinates(sensor2_pos_x, sensor2_pos_y)
	sensor3_pos_x = np.cos(rotation + sensor3_rotation)*sensor_distance + pos_x
	sensor3_pos_y = np.sin(rotation + sensor3_rotation)*sensor_distance + pos_y
	sensor3_position = position_to_coordinates(sensor3_pos_x, sensor3_pos_y)
	sensor4_pos_x = np.cos(rotation + sensor4_rotation)*sensor_distance + pos_x
	sensor4_pos_y = np.sin(rotation + sensor4_rotation)*sensor_distance + pos_y
	sensor4_position = position_to_coordinates(sensor4_pos_x, sensor4_pos_y)

def checkEdge():
	coords = None

	if sensor1 and sensor1_position != None:
		coords = sensor1_position
	if sensor2 and sensor2_position != None:
		coords = sensor2_position
	if sensor3 and sensor3_position != None:
		coords = sensor3_position
	if sensor4 and sensor4_position != None:
		coords = sensor4_position

	if coords not in edge_coordinates and coords != None:
		edge_coordinates.append(coords)


def position_to_coordinates(pos_x, pos_y):
	coord_x = int(np.floor((pos_x - origin_x) / resolution))
	coord_y = int(np.floor((pos_y - origin_y) / resolution))
	return coord_x, coord_y

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
		changed_map[y-10:y+10][x-10:x+10] = 100
	changed_map = tuple(changed_map.flatten())
	changed_grid.header = old_grid.header
	changed_grid.info = old_grid.info
	changed_grid.data = changed_map

def outOfBounds(coords):
	offset_x = 200
	offset_y = 150
	square_height = 50
	square_width = 60
	x, y = coords
	if x > square_width + offset_x or x < 0 + offset_x or y > square_height + offset_y or y < 0 + offset_y:
		if coords not in edge_coordinates:
			edge_coordinates.append(coords)
		return True
	else: return False


def checkSensorBounds():
	if sensor1_position != None and sensor2_position != None and sensor3_position != None and sensor4_position != None:
		a = outOfBounds(sensor1_position)
		b = outOfBounds(sensor2_position)
		c = outOfBounds(sensor3_position)
		d = outOfBounds(sensor4_position)
		return a or b or c or d
	else: return False
	'''
	if sensor1_position != None:
		return outOfBounds(sensor1_position)
	if sensor2_position != None:
		return outOfBounds(sensor2_position)
	if sensor3_position != None:
		return outOfBounds(sensor3_position)
	if sensor4_position != None:
		return outOfBounds(sensor4_position)
	'''

if __name__ == '__main__':
	rospy.init_node('mapChanger', anonymous=True)
	i = 0
	rate = rospy.Rate(200)
	pub = rospy.Publisher('/map', OccupancyGrid, queue_size=50)
	sensor_pub = rospy.Publisher('sensor_value', Bool, queue_size = 10)
	map_sub = rospy.Subscriber('/slammap', OccupancyGrid, saveMap)
	odom_sub = rospy.Subscriber('/odom', Odometry, getPose)
	while not rospy.is_shutdown():
		i = i+1
		if pos_x != None and pos_y != None and len(old_map) > 0 and rotation != None:
			if(checkSensorBounds() == True):
				sensor_pub.publish(True)
			setSensorPositions()
			checkEdge()
			changeMap(old_map)
			if(len(changed_map) > 0):
				pub.publish(changed_grid)
