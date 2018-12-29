#!/usr/bin/python
# Following this guide 
# https://tutorials-raspberrypi.com/raspberry-pi-ultrasonic-sensor-hc-sr04/
#Libraries
import RPi.GPIO as GPIO
import time
import rospy 
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import Range
import math

def distance(trig, echo):
    '''
    Calculates the distance of a sonar sensor
    arguments trig and echo are GPIO pins that
    are set to the valid mode 
    GPIO.setup(trig, GPIO.OUT)
    GPIO.setup(echo, GPIO.IN)
    '''
    # set Trigger to HIGH
    GPIO.output(trig, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(trig, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(echo) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(echo) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
 
    return distance/100.0
 

def create_ranged_message(distance):
    '''
    Create a ranged message for publishing sonar values
    The field of view takes the dual sonar implementation
    in account
    '''
    message = Range() 
    message.header.stamp = rospy.Time.now() 
    message.header.frame_id = 'base_footprint'
    message.radiation_type = 0
    message.field_of_view = math.pi/25# [rad] - 7.2 deg
    message.min_range = 0.05 # [m] closest 1 cm
    message.max_range = 3.50 # [m] furtheset 100 cm / 1m 
    message.range = distance - 0.05
    return message
 
if __name__ == '__main__':

    #GPIO Mode (BOARD / BCM)
    GPIO.setmode(GPIO.BCM)
    #set GPIO Pins
    trigger_1 = 17
    echo_1 = 27
    trigger_2 = 12
    echo_2 = 16
    #set GPIO direction (IN / OUT)
    GPIO.setup(trigger_1, GPIO.OUT)
    GPIO.setup(echo_1, GPIO.IN)
    GPIO.setup(trigger_2, GPIO.OUT)
    GPIO.setup(echo_2, GPIO.IN)

    rospy.init_node('sonar_publisher') 

    rate = rospy.Rate(5) # publish at 20 Hz
    sonar_pub = rospy.Publisher('/custom/sonar', Range, queue_size=1)

    try:
        while not rospy.is_shutdown():
            # publish the minimum of both sensor distances 
            # so the sonar acts as a double width sonar 
            # This is taken into account in the create message function
            d_array = []

            while(len(d_array) < 5):
                d1 = distance(trigger_1, echo_1) - 0.05
                d2 = distance(trigger_2, echo_2) - 0.05
                d = min(d1, d2)
                d_array.append(d)
            d_array = np.asarray(d_array)
            d_med = np.median(d_array)

            if d_med < 0.9 and max(d_array) - min(d_array) < 0.07:
                print(d_med)
                sonar_pub.publish(create_ranged_message(d_med))
            rate.sleep()
        GPIO.cleanup()

    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()
