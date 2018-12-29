from SquareMove import SquareMove
import rospy as ros
import time
from geometry_msgs.msg import Twist

class SquareMoveVel(SquareMove):
    """
    This class implements a open-loop square trajectory based on velocity control.     
    HOWTO:
     - Start the roscore (on the computer or the robot, depending on your configuration)
            $ roscore
     - Start the sensors on the turtlebot:
            $ roslaunch turtlebot3_bringup turtlebot3_robot.launch
     - Start this node on your computer:
            $ python move_square vel
    """

    def __init__(self):
        
        super(SquareMoveVel, self).__init__()

    def go_forward(self, duration, speed):

        # Get the initial time
        self.t_init = time.time()

        # Set the velocity forward and wait (do it in a while loop to keep publishing the velocity)
        while time.time() - self.t_init < duration and not ros.is_shutdown():

            msg = Twist()
            msg.linear.x = speed
            msg.angular.z = 0
            #now we forward the message to update the speed
            self.vel_ros_pub(msg)
            time.sleep(self.pub_rate)


    def turn(self, duration, ang_speed):

         # Get the initial time
        self.t_init = time.time()

        # Set the velocity forward and wait 2 sec (do it in a while loop to keep publishing the velocity)
        while time.time() - self.t_init < duration and not ros.is_shutdown():

            msg = Twist()
            msg.linear.x = 0
            msg.angular.z = ang_speed
            self.vel_ros_pub(msg)
            time.sleep(self.pub_rate)

    def move(self):

        self.go_forward(2, 0.5)
        self.turn(3.5, 0.5)
        self.go_forward(2, 0.5)
        self.turn(3.5, 0.5)
        self.go_forward(2, 0.5)
        self.turn(3.5, 0.5)
        self.go_forward(2, 0.5)
        self.stop_robot()
