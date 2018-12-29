import numpy as np
from geometry_msgs.msg import Point32

def position_to_coordinates(pos_x, pos_y, origin_x, origin_y, resolution):
	coord_x = int(np.floor((pos_x - origin_x) / resolution))
	coord_y = int(np.floor((pos_y - origin_y) / resolution))
	return coord_x, coord_y

def coordinates_to_position(coords, origin_x, origin_y, resolution):
	coord_x = coords[0]
	coord_y = coords[1]
	pos_x = coord_x * resolution + origin_x
	pos_y = coord_y * resolution + origin_y
	return pos_x, pos_y

def getSensorPosition(rotation, sensor_rotation, sensor_distance, pos_x, pos_y, origin_x, origin_y, resolution):
	sensor_pos_x = np.cos(rotation + sensor_rotation)*sensor_distance + pos_x
	sensor_pos_y = np.sin(rotation + sensor_rotation)*sensor_distance + pos_y
	sensor_position = position_to_coordinates(sensor_pos_x, sensor_pos_y, origin_x, origin_y, resolution)
	return sensor_position

def getSensorPositionPoint32(rotation, sensor_rotation, sensor_distance, pos_x, pos_y, origin_x, origin_y):
	sensor_position = Point32(0.0, 0.0, 0.0)
	sensor_position.x = np.cos(rotation + sensor_rotation)*sensor_distance# + pos_x
	sensor_position.y = np.sin(rotation + sensor_rotation)*sensor_distance# + pos_y
	return sensor_position

def offsetPoint32(sensor_position, origin_x, origin_y):
	corrected_position = Point32(sensor_position.x + origin_x, sensor_position.y + origin_y, 0)
	return corrected_position