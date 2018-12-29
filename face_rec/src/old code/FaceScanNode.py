#!/usr/bin/python

import rospy
import cv2
import time
import numpy as np
import datetime
from face_rec.msg import Person, PeopleList
from face_rec.srv import DetectFace, DetectFaceResponse
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
CAMERA_HORIZONTAL_ANGLE = 62.2
TIME_BEFORE_FORGOTTEN = 2 * 60


pos_x = 0
pos_y = 0
rotation = 0
#
# A Person is identified by his/her location x, y and an id.
#
class PersonPy:

    #
    #   Initialize a new person by giving him/her a unique id and a coordinate pair
    #
    def __init__(self, id_nr, coord):
        self.x, self.y = coord
        self.id_nr = id_nr
        self.last_seen = datetime.datetime.now()

    #
    #   Transform this Python object to a type ROS can interpret and publish
    #
    def get_message(self):
        p = Person()
        p.x = self.x
        p.y = self.y
        p.id = self.id_nr
        return p

#
#   This node-type can keep track of all the people it has seen in the world, via the photo's it takes
#
class FaceRecogNode:
    
    #
    #   This node needs to be able to capture a frame and analyze it via the FaceRecognizer class in another file
    #
    def __init__(self):
        self.recognized = {}

        rospy.wait_for_service('face_rec')
        self.recognizer = rospy.ServiceProxy('face_rec', DetectFace)
        self.bridge = CvBridge()
        # Try and secure a camera source
        # If the picamera import fails, we fall back to the CV2 camera
        try:
            import picamera
            import picamera.array
            self.cv2 = False

            print("+" * 100)
            print("Using: PiCamera")
            print("+" * 100)
        except:
            self.cap = cv2.VideoCapture(0)
            self.cv2 = True

            print("+" * 100)
            print("Using: CV2 Camera")
            print("+" * 100)

    #
    #   This method is called periodically and takes an image, before updating the internal state of this node
    #   New persons are added, and previous coordinates are updated
    #
    def take_image_and_recog(self):

        # Take an image on both dev and pi
        def take_image():
            if self.cv2:
                frame = self.cap.read()[1]
                return frame
            else:
                with picamera.PiCamera() as picam:
                    picam.start_preview()
                    time.sleep(2)
                    with picamera.array.PiRGBArray(picam) as stream:
                        picam.capture(stream, format='rgb')
                        return stream.array

        def get_percentage_in_frame(location):
            return float(location[0]) / 640.0

        def determine_position(percentage):
            import math
            percentage -= 0.5
            dy = math.sin(percentage * math.pi)
            dx = 1
            
            return pos_x + dx, pos_y + dy 
            
        frame = take_image()
        solution = self.recognizer(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

        # If new person, add to list of recognized persons
        if solution.ids and solution.positions:
            locations, ids = solution.positions, solution.ids
            for location, id_nr in zip(locations, ids):
                x,y = location.x, location.y

                if id_nr in self.recognized:
                    known = self.recognized[id_nr]
                    known.x, known.y = determine_position(get_percentage_in_frame((x,y)))
                    known.last_seen = datetime.datetime.now()
                    self.recognized[id_nr] = known
                else:
                    personPy = PersonPy(id_nr, determine_position(get_percentage_in_frame((x, y)))) 
                    self.recognized[id_nr] = personPy

        # Check if person is seen in recent times 
        for index, person in self.recognized.items():
            if abs((person.last_seen - datetime.datetime.now()).total_seconds()) > TIME_BEFORE_FORGOTTEN:
                print("Goodbye Mr/Ms {}".format(index))
                del self.recognized[index]
    

    #
    # Transform the knowledge from this node type into a type that ROS can understand
    #
    def get_all_seen(self):
        lis = PeopleList()
        lis.people = [person.get_message() for person in self.recognized.values()]
        return lis

# This code gets run on a loop. This runs next to the main thread of this ROS node
def main_loop(_):
    faceRecogNode.take_image_and_recog()
    publisher.publish(faceRecogNode.get_all_seen())
   

import tf 
def getPose(odom):
    pos_x = odom.pose.pose.position.x
    pos_y = odom.pose.pose.position.y
    (roll, pitch, rotation) = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])




if  __name__ == '__main__':
    rospy.init_node('FacialRecoqMapping')

    publisher = rospy.Publisher('people', PeopleList, queue_size=1)
    odom_sub = rospy.Subscriber('odom', Odometry, getPose)

    faceRecogNode = FaceRecogNode()

    print('init succeeded') 
    timer = rospy.Timer(rospy.Duration(3), main_loop)
    

    # Every 1 second (fastest rate possible), check if the program is shutdown or not
    checkRate = rospy.Rate(1)
    while not rospy.is_shutdown():
        checkRate.sleep()

    timer.shutdown()

