#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
import roslaunch
import numpy as np

old_map = []
old_grid = OccupancyGrid()
changed_map = []
changed_grid = OccupancyGrid()

def saveMap(grid):
	global old_map
	global old_grid
	old_map = grid.data
	old_grid = grid

def changeMap(map):
	global changed_map
	global changed_grid
	changed_map=list(map)
	map_width = old_grid.info.width
	w=0
	h=0
	for i in range(len(changed_map)):
		w+=1
		if i%map_width == map_width-1:
			h += 1
			w = 0
		if w > 200 and w < 206 and h > 100 and h < 200:
			changed_map[i] = 100
	changed_grid.header = old_grid.header
	changed_grid.info = old_grid.info
	changed_grid.data = tuple(changed_map)


if __name__ == '__main__':
	rospy.init_node('mapChanger', anonymous=True)
	rate = rospy.Rate(50)
	pub = rospy.Publisher('map', OccupancyGrid, queue_size=10)
	sub = rospy.Subscriber('slammap', OccupancyGrid, saveMap)

	while not rospy.is_shutdown():
		changeMap(old_map)
		if(len(changed_map) > 0):
			pub.publish(changed_grid)

		rate.sleep()