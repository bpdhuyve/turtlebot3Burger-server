#!/usr/bin/python
from std_msgs.msg import Bool
import rospy

mnm_bool = False
start = 0

def set_mnm_bool():
	return True


if __name__ == '__main__':
	global mnm_bool
	rospy.init_node('fake_mnm_pub')

	mnm_pub = rospy.Publisher('/custom/cliff_sensor_center_front', Bool, queue_size = 1)

	start = rospy.Time.now()

	while not rospy.is_shutdown():
		mnm_bool = set_mnm_bool()
		mnm_pub.publish(mnm_bool)
