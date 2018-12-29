#!/usr/bin/env python
import rospy
import roslaunch
import time

if __name__ == '__main__':
	rospy.init_node('piglet', anonymous=True)
	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
	roslaunch.configure_logging(uuid)

	path_slam = "/home/joppe/turtlebot_ws/src/turtlebot3/turtlebot3_slam/launch/turtlebot3_slam.launch"
	path_gazebo = "/home/joppe/turtlebot_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_office.launch"

	launch = roslaunch.parent.ROSLaunchParent(uuid, [path_slam, path_gazebo])

	launch.start()
	rospy.sleep(3)
	rospy.spin()
