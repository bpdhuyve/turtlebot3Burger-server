import numpy as np
from geometry_msgs.msg import Point32

def position_to_coordinates(pos_x, pos_y, origin_x, origin_y, resolution):
	'''
	Method that calculates the position of the robot on the slam map, based on its odometry.
	origin_x, origin_y and resolution are properties of the slam occupancygrid.
	'''
	coord_x = int(np.floor((pos_x - origin_x) / resolution))
	coord_y = int(np.floor((pos_y - origin_y) / resolution))
	return coord_x, coord_y

def coordinates_to_position(coords, origin_x, origin_y, resolution):
	'''
	Method that calculates the position of point in the odometry frame, based on a coordinate on the slam map.
	origin_x, origin_y and resolution are properties of the slam occupancygrid.
	'''
	coord_x = coords[0]
	coord_y = coords[1]
	pos_x = coord_x * resolution + origin_x
	pos_y = coord_y * resolution + origin_y
	return pos_x, pos_y

def getSensorPosition(rotation, sensor_rotation, sensor_distance, pos_x, pos_y, origin_x, origin_y, resolution):
	'''
	Method that calculates the position of a sensor based its rotation and distance to the robot.
	The returned position are coordinates on the slam map, the input positions are based on odometry of the robot.
	origin_x, origin_y and resolution are properties of the slam occupancygrid.
	'''
	sensor_pos_x = np.cos(rotation + sensor_rotation)*sensor_distance + pos_x
	sensor_pos_y = np.sin(rotation + sensor_rotation)*sensor_distance + pos_y
	sensor_position = position_to_coordinates(sensor_pos_x, sensor_pos_y, origin_x, origin_y, resolution)
	return sensor_position
