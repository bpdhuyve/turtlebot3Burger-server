#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
import roslaunch
import numpy as np

def listenMap():
	rospy.init_node('mapListener', anonymous=True)
	rospy.Subscriber('map', OccupancyGrid, callback)
	rospy.spin()


def callback(map):
	test = map.info
	print(test)

if __name__ == '__main__':
	listenMap()
