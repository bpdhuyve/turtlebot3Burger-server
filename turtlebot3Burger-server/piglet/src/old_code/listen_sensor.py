#!/usr/bin/python
import std_msgs.msg
import rospy

def printSensor(sensordata):
	print(sensordata)

if __name__ == '__main__':
	ropsy.Rate(10)
	rospy.init_node('sensor_listener')

	sub_right = rospy.Subscriber('/custom/cliff_sensor_right', std_msgs.msg.Bool, printSensor)
	sub_left = rospy.Subscriber('/custom/cliff_sensor_left', std_msgs.msg.Bool, printSensor)
	sub_front_right = rospy.Subscriber('/custom/cliff_sensor_front_right', std_msgs.msg.Bool, printSensor)
	sub_front_left = rospy.Subscriber('/custom/cliff_sensor_front_left', std_msgs.msg.Bool, printSensor)
	sub_center_front = rospy.Subscriber('/custom/cliff_sensor_center_front', std_msgs.msg.Bool, printSensor)
	sub_center_back = rospy.Subscriber('/custom/cliff_sensor_center_back', std_msgs.msg.Bool, printSensor)

	rospy.spin()
