#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
import roslaunch
import numpy as np

def create_map():
    test_map = OccupancyGrid()
    test_map.info.resolution = 1.0 
    test_map.info.width = 10
    test_map.info.height = 10
    test_map.info.origin.position.x = 0.0 
    test_map.info.origin.position.y = -7.0 
    test_map.info.origin.position.z = 2.0 
    test_map.info.origin.orientation.x = 3.0 
    test_map.info.origin.orientation.y = 4.0 
    test_map.info.origin.orientation.z = 5.0 
    test_map.info.origin.orientation.w = 6.0 
    test_map.data = []
    for i in range(0, 100):
        test_map.data.append(100)
    print test_map
 
    map_pub.publish(test_map);
    print("createmap")
    return 

if __name__ == '__main__':
	global map_pub
	map_pub = rospy.Publisher('map', OccupancyGrid, queue_size=10)
	rospy.init_node('turtlebot_map', anonymous=True)
	r = rospy.Rate(10)
	while not rospy.is_shutdown():
		create_map()
		r.sleep()