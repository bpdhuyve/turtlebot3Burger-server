#!/usr/bin/env python
import rospy
from nav_msgs.msg import MapMetaData
import roslaunch

def print_width(data):
	print(data.height)

def listener():
	rospy.init_node('listener', anonymous=True)
	print('loop')
	rospy.Subscriber('map_metadata', MapMetaData, print_width)
	rospy.spin()

if __name__ == '__main__':
	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
	roslaunch.configure_logging(uuid)

	path_slam = "/home/joppe/turtlebot_ws/src/turtlebot3/turtlebot3_slam/launch/turtlebot3_slam.launch"
	path_gazebo = "/home/joppe/turtlebot_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_table.launch"

	launch = roslaunch.parent.ROSLaunchParent(uuid, [path_slam, path_gazebo])

	launch.start()
	listener()