#!/usr/bin/env python
import math
import rospy as ros
import sys
import time

from turtlebot3_msgs.msg import SensorState
# from sensors.msg import CliffSensors
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class Navigation(object):
    """
    This class is an abstract class to control a square trajectory on the turtleBot.
    It mainly declares and subscribes to ROS topics in an elegant way.
    """

    def __init__(self):
	print("starting init")
        # Declare ROS subscribers and publishers
        self.node_name = "navigation"
        self.odom_sub_name = "/odom"
	self.odom_imu_sub_name = "/imu_odom"

        self.cliff_sensor_left_sub_name = "/custom/cliff_sensor_left"
        self.cliff_sensor_right_sub_name = "/custom/cliff_sensor_right"
        self.cliff_sensor_front_left_sub_name = "/custom/cliff_sensor_front_left"
        self.cliff_sensor_front_right_sub_name = "/custom/cliff_sensor_front_right"
        self.cliff_sensor_center_front_sub_name = "/custom/cliff_sensor_center_front"
        self.cliff_sensor_center_back_sub_name = "/custom/cliff_sensor_center_back"
	self.sensor_state_sub_name = "/sensor_state"

	self.vel_pub_name = "/cmd_vel"

        self.odometry_sub = None
	self.odometry_imu_sub = None

	self.cliff_sensor_l_sub = None
        self.cliff_sensor_r_sub = None
        self.cliff_sensor_fl_sub = None
        self.cliff_sensor_fr_sub = None
        self.cliff_sensor_cf_sub = None
        self.cliff_sensor_cb_sub = None
	self.sensor_state_sub = None

        self.vel_pub = None

        # ROS params
        self.pub_rate = 0.01
        self.queue_size = 2

        # Variables containing the sensor information that can be used in the main program
        self.odom_pose = None
	self.odom_imu_pose = None
	self.sensors = {"ir_left": 0, "ir_right": 0, "ir_front_left": 0, "ir_front_right": 0, "ir_center_front": 0, "ir_center_back": 0, "sonar": 0}
	
	# conclusie: hij roept de cliff_sensor_..._sub niet op die waarden moet aannemen. Hoe lees ik waarden in?!
	# ook wordt de pose dus niet ingelezen omdat hij die functie ook niet voor de eerste keer zal oproepen in start_ros
	print("starting ROS")
	self.start_ros()
	print(self.sensors)
	self.move_indep_using_sensors()

    def start_ros(self):
	print("starting start_ros")
        # Create a ROS node with a name for our program
        ros.init_node(self.node_name, log_level=ros.INFO)

        # Create subscribers
	# for ir sensors
        self.cliff_sensor_l_sub = ros.Subscriber(self.cliff_sensor_left_sub_name, Bool, callback=self.cliff_sensor_left_sub, queue_size=self.queue_size)
        self.cliff_sensor_r_sub = ros.Subscriber(self.cliff_sensor_right_sub_name, Bool, callback=self.cliff_sensor_right_sub, queue_size=self.queue_size)
        self.cliff_sensor_fl_sub = ros.Subscriber(self.cliff_sensor_front_left_sub_name, Bool, callback=self.cliff_sensor_front_left_sub, queue_size=self.queue_size)
        self.cliff_sensor_fr_sub = ros.Subscriber(self.cliff_sensor_front_right_sub_name, Bool, callback=self.cliff_sensor_front_right_sub, queue_size=self.queue_size)
        self.cliff_sensor_cf_sub = ros.Subscriber(self.cliff_sensor_center_front_sub_name, Bool, callback=self.cliff_sensor_center_front_sub, queue_size=self.queue_size)
        self.cliff_sensor_cb_sub = ros.Subscriber(self.cliff_sensor_center_back_sub_name, Bool, callback=self.cliff_sensor_center_back_sub, queue_size=self.queue_size)
	# for sonar sensor
        self.sensor_state_sub = ros.Subscriber(self.sensor_state_sub_name, SensorState, callback=self.sonar_sensors_sub, queue_size=self.queue_size)
	
	# for odometry
        self.odometry_sub = ros.Subscriber(self.odom_sub_name, Odometry, callback=self.odom_sub, queue_size=10)
	# just add this, it's not stupid if it works
	self.odometry_imu_sub = ros.Subscriber(self.odom_imu_sub_name, Odometry, callback=self.odom_imu_sub, queue_size=self.queue_size)
        # Define a callback to stop the robot when we interrupt the program (CTRL-C)
        ros.on_shutdown(self.stop_robot)

        # Create publisher
        self.vel_pub = ros.Publisher(self.vel_pub_name, Twist, queue_size=self.queue_size)

    def stop_robot(self):

        # Get the initial time
        self.t_init = time.time()

        # We publish for a second to be sure the robot receive the message
        while time.time() - self.t_init < 1:
            self.vel_ros_pub(Twist())
            time.sleep(self.pub_rate)

        sys.exit("The process has been interrupted by the user!")

    def obstacle_detection(self):
        # True if cliff 
        ret = self.sensors["ir_left"] or self.sensors["ir_right"] or self.sensors["ir_front_right"] or self.sensors["ir_front_left"] or self.sensors["ir_center_back"] or self.sensors["ir_center_front"]
        return ret

    def move(self, d, speed=-0.5): # Speed is negative because we put the sensors wrong
        """ To be surcharged in the inheriting class"""
        if self.odom_pose:
            x_init = self.odom_pose.position.x
            y_init = self.odom_pose.position.y

            # Set the velocity forward until distance is reached
            dist = math.sqrt((self.odom_pose.position.x - x_init) ** 2 + (self.odom_pose.position.y - y_init) ** 2) 
            print(dist) 
            while dist < d and not ros.is_shutdown():
                if not self.obstacle_detection():
                    print("NOT CLIFF")
                    msg = Twist()
                    msg.linear.x = speed
                    msg.angular.z = 0
                    self.vel_ros_pub(msg)
                else:
                    print("CLIFF")
                    self.turn_of(math.pi / 16)
                time.sleep(self.pub_rate)

            while not ros.is_shutdown():
                time.sleep(1)
            return True
        else: 
            return False

    def move_indep_using_sensors(self):
        # Wait that our python program has received its first messages
	# define turning angle of 10 degrees
	print("starting move independantly using ir sensors")

	while self.odom_pose is None and not ros.is_shutdown():
		print("while odom_pose is None")
               	time.sleep(0.1)

    	# Implement main instructions
	theta = math.pi/18
	# distance to move in meter
	d = 0.02
	v = -0.05
	a = -0.3
    	while not ros.is_shutdown():
    	    	# Implement main instructions
		# cliff_sensors_sub()
		SL = self.sensors["ir_left"]
		SR = self.sensors["ir_right"]
		FL = self.sensors["ir_front_left"]
        	FR = self.sensors["ir_front_right"]
		CB = self.sensors["ir_center_back"]
		print("sensors:",SL,SR,FL,FR,CB)
		print(self.odom_pose)
		if(not(SL) and not(FL) and not(SR)):
		# move forward
			print("front: nothing")
			self.move_f(d,v)
		elif(SL and FL and SR):
		# move backward
			print("front: all")
			self.move_f(d,-v)
		elif(SL and not(FL) and not(SR)):
		# turn clockwise
			print("front: left side")
			self.turn_of(theta,a)
		elif(SL and FL and not(SR)):
		# move backwards and turn counter clockwise
			print("front: left side and front")
			self.move_f(d,-v)
			self.turn_of(theta,-a)
		elif(SL and not(FL) and SR):
		# move backwards
			print("front: left side and right side")
			self.move_f(d,-v)
		elif(not(SL) and FL and not(SR)):
		#turn theta because needs to detect which sort of corner it stands
			print("front: front")
			self.turn_of(theta,a)
		elif(not(SL) and not(FL) and SR):
		# turn counterclockwise
			print("front: right side")
			self.turn_of(theta,-a)
		elif(not(SL) and FL and SR):
		# move backwards and turn clockwise
			print("front: left side and front")
			self.move_f(d,-v)
			self.turn_of(theta,a)

    def move_f(self, d, speed=-0.1): # Speed is negative because we put the sensors "wrong"
	print("starting move_f")
        # define turning angle of 10 degrees
	theta = math.pi/18
	# distance to move in meter is "d"
	# velocity to move is "speed"

	# publish speed
	msg = Twist()
        msg.linear.x = speed
        msg.angular.z = 0
        self.vel_ros_pub(msg)
        time.sleep(self.pub_rate)
	print("move_f published speed", msg)

	# definieer CB
	CB = self.sensors["ir_center_back"]

        if self.odom_pose:
		print
		x_init = self.odom_pose.position.x
		y_init = self.odom_pose.position.y

		# Set the velocity forward until distance is reached
		dist = math.sqrt((self.odom_pose.position.x - x_init) ** 2 + (self.odom_pose.position.y - y_init) ** 2) 
		print(dist) 
		while dist < d and not ros.is_shutdown():
			print("move_f: into while")
			# when moving backwards near cliff, then move forward
			if(speed > 0 and CB):
				print("move_f: correction for cliff at back")
				move_f(d,-speed);				
			# when moving
			else:
				print("move_f: just moving")
			    	msg = Twist()
			    	msg.linear.x = speed
			    	msg.angular.z = 0
			    	self.vel_ros_pub(msg)
				time.sleep(self.pub_rate)
			dist = math.sqrt((self.odom_pose.position.x - x_init) ** 2 + (self.odom_pose.position.y - y_init) ** 2) 
		    	print(dist)
			print("move_f: out of while")
        else: 
            print("move_f not obtained odom_pose")

    def turn_of(self, a, ang_speed=0.1):
        # Convert the orientation quaternion message to Euler angles
        a_init = self.get_z_rotation(self.odom_pose.orientation)
        # Set the angular velocity forward until angle is reached
        while (abs(self.get_z_rotation(self.odom_pose.orientation) - a_init) < a and not ros.is_shutdown()):
            print([self.get_z_rotation(self.odom_pose.orientation), a_init, self.get_z_rotation(self.odom_pose.orientation) - a_init, a])
            msg = Twist()
            msg.angular.z = ang_speed
            msg.linear.x = 0
            self.vel_ros_pub(msg)
            time.sleep(self.pub_rate)
	    
	    

    def get_z_rotation(self, orientation):
        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        return yaw

    def odom_sub(self, msg):
	print("odom_sub at ",time.asctime(time.localtime(time.time())))
        self.odom_pose = msg.pose.pose
        print(self.odom_pose.position)
        print(self.odom_pose.orientation)
	print("out of odom_sub")

    def odom_imu_sub(self, msg):

        self.odom_imu_pose = msg.pose.pose
        

    def vel_ros_pub(self, msg):

        self.vel_pub.publish(msg)

    # ------------------------------------------------------
    # All the ir callbacks!
    # ------------------------------------------------------
    def cliff_sensor_left_sub(self, msg):
        self.sensors["ir_left"] = msg.data

    def cliff_sensor_right_sub(self, msg):
        self.sensors["ir_right"] = msg.data

    def cliff_sensor_front_left_sub(self, msg):
        self.sensors["ir_front_left"] = msg.data

    def cliff_sensor_front_right_sub(self, msg):
        self.sensors["ir_front_right"] = msg.data

    def cliff_sensor_center_front_sub(self, msg):
        self.sensors["ir_center_front"] = msg.data

    def cliff_sensor_center_back_sub(self, msg):
        self.sensors["ir_center_back"] = msg.data

    def sonar_sensors_sub(self, msg):
        self.sensors["sonar"] = msg.sonar
    # ------------------------------------------------------


if __name__ == '__main__':

    # Choose the example you need to run in the command line
    print("starting main") 
    Navigation()
    if len(sys.argv) > 1:

        if sys.argv[1] == "odom":
	    print("LOL")
            r = Navigation()
        else:
            sys.exit(-1)

    else:
        sys.exit(-1)

    # Listen and Publish to ROS + execute moving instruction
    print("done") 
