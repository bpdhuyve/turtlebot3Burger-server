#!/usr/bin/python
import RPi.GPIO as GPIO
import time
import signal
import sys
import rospy
import std_msgs.msg
from turtlebot3_msgs.msg import SensorState
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32

class PointCloudPublisher(object): 

    def __init__(self): 
        '''
        This class does not take any parameters
        Setup of the different pins is done in the init
        '''
        # Read input from pin
        self.input_pin_r = 10 # back right
        self.input_pin_l = 9  # back left
        self.input_pin_front_r = 13 # front right
        self.input_pin_front_l = 26 # front left
        self.input_pin_center_front = 6 # center front = mnm grabbing detection
        self.input_pin_center_back = 5 # center back
        self.sonar_trigger = 17 # sonar trigger
        self.sonar_echo = 27 # sonar echo

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.input_pin_r, GPIO.IN)
        GPIO.setup(self.input_pin_l, GPIO.IN)
        GPIO.setup(self.input_pin_front_r, GPIO.IN)
        GPIO.setup(self.input_pin_front_l, GPIO.IN)
        GPIO.setup(self.input_pin_center_front, GPIO.IN)
        GPIO.setup(self.input_pin_center_back, GPIO.IN)
        GPIO.setup(self.sonar_trigger, GPIO.OUT)
        GPIO.setup(self.sonar_echo, GPIO.IN)

        self.sensors = {
            "left": False,
            "right": False,
            "front_left": False,
            "front_right": False,
            "front_center": False,
            "back_center": False,
            "sonar": 1000
        }

        signal.signal(signal.SIGINT, self.__signal_handler)
        self.nav_goal_pub = rospy.Publisher("/PointCloud",PointCloud, queue_size = 1)

    def update_sensors(self): 
        '''
        Read the values from all input pins and store the state in a class dictionary
        self.sensors
        '''
        # update values
        self.sensors['front_right'] = GPIO.input(self.input_pin_front_r)
        self.sensors['front_left'] = GPIO.input(self.input_pin_front_l)
        self.sensors['right'] = GPIO.input(self.input_pin_r)
        self.sensors['left'] = GPIO.input(self.input_pin_l)
        self.sensors['front_center'] = GPIO.input(self.input_pin_center_front)
        self.sensors['back_center'] = GPIO.input(self.input_pin_center_back)
        # sonar 
        self.sensors['sonar'] = self.__calculate_sonar_distance()

    def __calculate_sonar_distance(self): 
        '''
        Calculate the distance of the (single) sonar implementation
        - Is not used anymore since the sonars were added on a different
        navigation layer (ranged layer) 
        '''
        # Following this guide 
        # https://tutorials-raspberrypi.com/raspberry-pi-ultrasonic-sensor-hc-sr04/
        # set Trigger to HIGH
        GPIO.output(self.sonar_trigger, True)
     
        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(self.sonar_trigger, False)
     
        StartTime = time.time()
        StopTime = time.time()
     
        # save StartTime
        while GPIO.input(self.sonar_echo) == 0:
            StartTime = time.time()
     
        # save time of arrival
        while GPIO.input(self.sonar_echo) == 1:
            StopTime = time.time()
     
        # time difference between start and arrival
        TimeElapsed = StopTime - StartTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2
     
        return distance/100.0 # return in [m] instead of [cm]

    def __signal_handler(self, sig, frame):
        '''
        Callback so that keyboard interrupts work
        '''
        print('Exit')
        sys.exit(o)

    def publish_pointcloud(self): 
        '''
        Publish the sensor state as a pointcloud to the pointcloud topic 
        So they can be taken into account by the map
        '''
        #print(self.sensors)
        
        offset = 0.06 # offset 5 cm 
        scan = PointCloud()
        scan.header.frame_id = "base_link"
        scan.header.stamp = rospy.Time.from_sec(rospy.Time(0).to_sec())

        # Create pointclouds from the sensor data 
        if self.sensors["left"]: 
            scan.points.append(Point32(-0.16,0.06, 0.0))
        if self.sensors["right"]: 
            scan.points.append(Point32(-0.16,- 0.06, 0.0))
        if self.sensors["front_left"]: 
            scan.points.append(Point32(0.12,0.06, 0.0))
        if self.sensors["front_right"]: 
            scan.points.append(Point32(0.12, -0.06, 0.0))
        #if self.sensors["front_center"]: 
        #    scan.points.append(Point32(0.14, -0.02, 0.0))
        if not self.sensors["back_center"]: 
            scan.points.append(Point32(-0.06 ,-0.02 ,0.0))

        # only publish the pointcloud if any of the senors are triggered
        if (self.sensors["left"] or 
            self.sensors["right"] or 
            self.sensors["front_left"] or 
            self.sensors["front_right"] or 
            not self.sensors["back_center"] or
            self.sensors["sonar"] < 1.0):

            self.nav_goal_pub.publish(scan)

if __name__ == '__main__':
    rospy.init_node('pc_publisher')
    pc_publisher = PointCloudPublisher()
    # Publish 5 times per second
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        pc_publisher.update_sensors()
        pc_publisher.publish_pointcloud()
        rate.sleep()
