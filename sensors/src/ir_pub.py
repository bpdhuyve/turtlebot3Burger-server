#!/usr/bin/python

import RPi.GPIO as GPIO
import time
import signal
import sys
import rospy
import std_msgs.msg

# Allow for keyboard interrupts
def signal_handler(sig, frame):
    print('Exit')
    sys.exit(o)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('ir_publisher')

    # Publish all cliff sensor information 
    # on separate topics for the use of the 
    # backoff mechanism in the top navigation node 
    pub_right = rospy.Publisher(
        '/custom/cliff_sensor_right',
        std_msgs.msg.Bool,
        queue_size=1
    )
    pub_left = rospy.Publisher(
        '/custom/cliff_sensor_left',
        std_msgs.msg.Bool,
        queue_size=1
    )
    pub_front_right = rospy.Publisher(
        '/custom/cliff_sensor_front_right',
        std_msgs.msg.Bool,
        queue_size=1
    )
    pub_front_left = rospy.Publisher(
        '/custom/cliff_sensor_front_left',
        std_msgs.msg.Bool,
        queue_size=1
    )
    pub_center_front = rospy.Publisher(
        '/custom/cliff_sensor_center_front',
        std_msgs.msg.Bool,
        queue_size=1
    )
    pub_center_back = rospy.Publisher(
        '/custom/cliff_sensor_center_back',
        std_msgs.msg.Bool,
        queue_size=1
    )

    # Read input from pin
    input_pin_r = 10 
    input_pin_l = 9 
    input_pin_front_r = 13
    input_pin_front_l = 26

    # More IR 
    input_pin_center_front = 6
    input_pin_center_back = 5

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(input_pin_r, GPIO.IN)
    GPIO.setup(input_pin_l, GPIO.IN)
    GPIO.setup(input_pin_front_r, GPIO.IN)
    GPIO.setup(input_pin_front_l, GPIO.IN)
    GPIO.setup(input_pin_center_front, GPIO.IN)
    GPIO.setup(input_pin_center_back, GPIO.IN)

    # Publish 5 times per second
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():

        # publish value
        fr = std_msgs.msg.Bool() 
        fr.data = GPIO.input(input_pin_front_r)
        pub_front_right.publish(fr)

        fl = std_msgs.msg.Bool() 
        fl.data = GPIO.input(input_pin_front_l)
        pub_front_left.publish(fl) 

        r = std_msgs.msg.Bool() 
        r.data = GPIO.input(input_pin_r)
        pub_right.publish(r)

        l = std_msgs.msg.Bool() 
        l.data = GPIO.input(input_pin_l)
        pub_left.publish(l)
        rate.sleep()

        cf = std_msgs.msg.Bool() 
        cf.data = GPIO.input(input_pin_center_front)
        pub_center_front.publish(cf)
        rate.sleep()

        cb = std_msgs.msg.Bool() 
        cb.data = GPIO.input(input_pin_center_back)
        pub_center_back.publish(cb)
        rate.sleep()
