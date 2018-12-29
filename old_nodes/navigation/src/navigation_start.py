#!/usr/bin/env python
import math
import rospy as ros
import sys
import time

from turtlebot3_msgs.msg import SensorState
from sensors.msg import CliffSensors
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class Navigation(object):
    """
    This class is used to control a square trajectory on the turtleBot and will be used by the start script to make an initial map of the table.
    """

    def __init__(self):

        # Declare ROS subscribers and publishers
        self.node_name = "navigation_start"
        self.odom_sub_name = "/odom"
        self.cliff_sub_name = "/cliff"

        self.cliff_sensor_left_sub_name = "/custom/cliff_sensor_left"
        self.cliff_sensor_right_sub_name = "/custom/cliff_sensor_right"
        self.cliff_sensor_front_left_sub_name = "/custom/cliff_sensor_front_left"
        self.cliff_sensor_front_right_sub_name = "/custom/cliff_sensor_front_right"
        self.cliff_sensor_center_front_sub_name = "/custom/cliff_sensor_center_front"
        self.cliff_sensor_center_back_sub_name = "/custom/cliff_sensor_center_back"

        self.sensor_state_sub_name = "/sensor_state"
	self.odometry_sub = None
        self.cliff_sub = None
        self.vel_pub_name = "/cmd_vel"
        self.vel_pub = None
        
        # ROS params
        self.pub_rate = 0.1
        self.queue_size = 2

        # Variables containing the sensor information that can be used in the main program
        self.odom_pose = None
        self.sensors = {}

    def start_ros(self):

        # Create a ROS node with a name for our program
        ros.init_node(self.node_name, log_level=ros.INFO)

	# Define a callback to stop the robot when we interrupt the program (CTRL-C)
        ros.on_shutdown(self.stop_robot)

        # Create subscribers
        ros.Subscriber(self.cliff_sensors_sub_name, CliffSensors, callback=self.cliff_sensors_sub, queue_size=self.queue_size)
        ros.Subscriber(self.sensor_state_sub_name, SensorState, callback=self.sonar_sensors_sub, queue_size=self.queue_size)
        self.odometry_sub = ros.Subscriber(self.odom_sub_name, Odometry, callback=self.odom_sub,
                                           queue_size=self.queue_size)

        # Create publisher
        self.vel_pub = ros.Publisher(self.vel_pub_name, Twist, queue_size=self.queue_size)

    def stop_robot(self):

        # Get the initial time
        self.t_init = time.time()

        # We publish for a second to be sure the robot receive the message
        while time.time() - self.t_init < 1 and not ros.is_shutdown():
            self.vel_ros_pub(Twist())
            time.sleep(self.pub_rate)

        sys.exit("The process has been interrupted by the user!")

    def move(self): # Speed is negative because we put the sensors wrong
        # Wait that our python program has received its first messages
    	while self.odom_pose is None and not ros.is_shutdown():
                time.sleep(0.1)

    	    	# Implement main instructions
		cliff_sensors_sub() #wat als msg?
		SL = self.sensors["ir_left"]
		SR = self.sensors["ir_right"]
		FL = self.sensors["ir_front_left"]
        	FR = self.sensors["ir_front_right"]
		move_of(-0.05)
		if(not(SL) and not(FR) and not(SR)):
			move_of(-0.10)
		elif(SL and not(FR) and not(SR)):
			turn_of(-math.pi/8)
		elif(SL and FR and not(SR)):
			turn_of(-math.pi/8)
		elif(SL and not(FR) and SR): # can not be in real world
			move_of(0.05) #just go backwards
		elif(not(SL) and FR and not(SR)):
			turn_of(math.pi/8) #turn pi/4 because needs to detect which sort of corner it stands
		elif(not(SL) and not(FR) and SR):
			turn_of(math.pi/8)
		elif(not(SL) and FR and SR):
			turn_of(math.pi/8)		
			
    def move_of(self, d, speed=-0.05):

    	    x_init = self.odom_pose.position.x
    	    y_init = self.odom_pose.position.y

    	    # Set the velocity forward until distance is reached
    	    while math.sqrt((self.odom_pose.position.x - x_init)**2 + \
                 (self.odom_pose.position.y - y_init)**2) < d and \
                 not ros.is_shutdown():

        	    sys.stdout.write("\r [MOVE] The robot has moved of " + \
                     "{:.2f}".format(math.sqrt((self.odom_pose.position.x -\
                     x_init)**2 + (self.odom_pose.position.y - y_init)**2)) + \
                     "m over " + str(d) + "m")
        	    sys.stdout.flush()

        	    msg = Twist()
        	    msg.linear.x = speed
        	    msg.angular.z = 0
        	    self.vel_ros_pub(msg)
        	    time.sleep(self.pub_rate)

    	    sys.stdout.write("\n")

    def get_z_rotation(self, orientation):
    	    (roll, pitch, yaw) = euler_from_quaternion([orientation.x, \
                                orientation.y, orientation.z, orientation.w])
    	    return yaw

    def turn_of(self, a, ang_speed=0.4):

    	    # Convert the orientation quaternion message to Euler angles
    	    a_init = self.get_z_rotation(self.odom_pose.orientation)

    	    # Set the angular velocity forward until angle is reached
    	    while (self.get_z_rotation(self.odom_pose.orientation) - a_init) < a and not ros.is_shutdown():

        	sys.stdout.write("\r [TURN] The robot has turned of " + "{:.2f}".format(self.get_z_rotation(self.odom_pose.orientation)- a_init) + "rad over {:.2f}".format(a) + "rad")
                sys.stdout.flush()
              	print (self.get_z_rotation(self.odom_pose.orientation) - a_init)

		msg = Twist()
		msg.angular.z = ang_speed
		msg.linear.x = 0
		self.vel_ros_pub(msg)
		time.sleep(self.pub_rate)

    	    sys.stdout.write("\n")

    def __odom_ros_sub(self, msg):
	self.odom_pose = msg.pose.pose

    def vel_ros_pub(self, msg):
	self.vel_pub.publish(msg)

    def odom_sub(self, msg):
        self.odom_pose = msg.pose.pose

    def vel_ros_pub(self, msg):
        self.vel_pub.publish(msg)

    def cliff_sensors_sub(self, msg):
        self.sensors["ir_left"] = msg.sensor_left
        self.sensors["ir_right"] = msg.sensor_right
        self.sensors["ir_front_left"] = msg.sensor_front_right
        self.sensors["ir_front_right"] = msg.sensor_front_left

    def sonar_sensors_sub(self, msg):
        self.sensors["sonar"] = msg.sonar


if __name__ == '__main__':

    # Choose the example you need to run in the command line
    print("starting main") 
    if len(sys.argv) > 1:

        if sys.argv[1] == "odom":
            r = Navigation()
        else:
            sys.exit(-1)

    else:
        sys.exit(-1)

    # Listen and Publish to ROS + execute moving instruction
    print("starting ROS")
    r.start_ros()
    print("starting move")
    r.move(10)
    print("done") 
