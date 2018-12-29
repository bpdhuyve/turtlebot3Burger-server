#!/usr/bin/python
import roslib; roslib.load_manifest('face_rec')
import rospy
from cv_bridge import CvBridge, CvBridgeError
from FaceRecognizer import FaceRecognizer
from geometry_msgs.msg import Point, Pose, PoseStamped, PointStamped
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import numpy as np
import copy 
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
import cv2
import datetime
from pyquaternion import Quaternion
import tf

# Time before the robot forgets a face
TIME_BEFORE_FORGOTTEN = 300
# Store the received image as a png
SHOW_RECEIVED_IMAGE = True

class RecognitionSubscriber(object):
    '''
    Class that subscribes on compute_faces_and_position
    receives images and computes the face id and target location from this
    and publishes this on the people_simple topic
    '''

    def __init__(self):
        '''
        This class does not take any arguments
        '''
        self.fr = FaceRecognizer()
        self.bridge = CvBridge()
        rospy.Subscriber(
            '/odom', 
            Odometry, 
            self.__odom_callback) 
    
        # Subscribe on the received images
        rospy.Subscriber(
            '/custom/compute_faces_and_positions', 
            Image, 
            self.__request_callback)
    
        # Pubish the computed information
        self.recognition_pub = rospy.Publisher(
            '/custom/people_simple', Float32MultiArray, queue_size=1) 

        self.position_publisher = rospy.Publisher(
            '/custom/people_positions', 
            PoseStamped, 
            queue_size=1,
            latch=True    
        )

        # Contineously keep track of the current odometry to calculate target location
        # in correspondance to odom
        self.odom = Odometry()

        # Keep lists of the recognized ids, locations and times 
        # lists are easy to remove a target id from all lists after a 
        # while and to check if id is present, ...
        self.recognized_ids = [] 
        self.recognized_locs = []
        self.recognized_times = []

    def publish_position(self, x,y, frame_id):
        '''
        Publish the position of a detected face on the map 
        arguments: x, y, frame_id (reference)  
        '''
        pose = PoseStamped() 
        pose.header.frame_id = frame_id
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        self.position_publisher.publish(pose) 
        
    def __request_callback(self, msg): 
        '''
        Callback if image is received, faces are detected and 
        added to the global array of people
        A message is sent with the ids, locations and face widths 
        in a multiarray.
        '''
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        if SHOW_RECEIVED_IMAGE:
            cv2.imwrite('rec_img.png', cv_image)
        ret = self.fr.recognize_people(cv_image)
        simple_msg = Float32MultiArray() 
        if ret: 
            print('new face detected') 
            locs, ids = ret
            for i, loc in zip(ids, locs): 
                if i in self.recognized_ids: 
                    index = self.recognized_ids.index(i) 
                    self.recognized_locs[index] = loc
                    self.recognized_times[index] = datetime.datetime.now() 
                else:
                    self.recognized_ids.append(i)
                    self.recognized_locs.append(loc)
                    self.recognized_times.append(datetime.datetime.now())
        else: 
            print('No faces detected') 

        # Publish the computed info on the simple_people topic 
        # as a multiarray. The structure is defined as [id, xpos, ypos, width] 
        # in a repeated fashion for multiple faces. 
        # The width might be used as a metric for how far the face is away 
        dim = MultiArrayDimension()
        dim.label = 'id_and_pos'
        dim.size = 3
        dim.stride = 0 
        dim2 = MultiArrayDimension()
        dim2.label = 'length_faces'
        dim2.size = len(self.recognized_ids)
        dim2.stride = 0 
        simple_msg.layout.dim = [dim, dim2]
        simple_msg.layout.data_offset = 0
        data = []
        # Add the recognized faces around the table to the message
        for i, loc in zip(self.recognized_ids, self.recognized_locs):
            width = self.__get_width(loc)
            x, y, valid = self.__calculate_people_positions(width) 
            data = np.concatenate((data, [i,x,y,width]), axis=0) 
        simple_msg.data = data
        print(simple_msg)
        # Publish the message
        self.recognition_pub.publish(simple_msg)

        # Check if person is seen in the last TIME_BEFORE_FORGOTTEN seconds and otherwise remove
        for index, time in enumerate(self.recognized_times):
            if abs((time - datetime.datetime.now()).total_seconds()) > TIME_BEFORE_FORGOTTEN:
                del self.recognized_ids[index]
                del self.recognized_locs[index]
                del self.recognized_times[index]

    def __odom_callback(self, msg): 
        '''
        Keep the odometry up to date
        '''
        self.odom = msg

    def __get_width(self, location):
        '''
        Helper function to calculate the width of a face
        '''
        return abs(location[3] - location[1])

    def __get_height(self, location):
        '''
        Helper function to calculate the height of a face
        '''
        return abs(location[0] - location[2])

    def __get_center(self, location):
        '''
        Helper function to calculate the center of a face
        '''
        def get_top_left(location):
            return location[3], location[0]

        def get_bottom_right(location):
            return location[1], location[2] 
        # Calculate the x, y position of the found person
        x_l, y_l = get_top_left(location)
        x_r, y_r = get_bottom_right(location)
        point = Point()
        point.x = (x_l + x_r) / 2.0
        point.y = (y_l + y_r) / 2.0
        point.z = 0 
        return point 

    def __calculate_people_positions(self, width):
        '''
        Calculate the peoples position based on the current odometry
        Assume that the face is straight ahead and the distance is 
        dependent on the face width 
        '''
        _,_,yaw = tf.transformations.euler_from_quaternion([
            self.odom.pose.pose.orientation.x,
            self.odom.pose.pose.orientation.y,
            self.odom.pose.pose.orientation.w,
            self.odom.pose.pose.orientation.z,
        ])
        print(yaw)
        print(self.odom)
        target_pose = copy.deepcopy(self.odom)
        #fac = 1.0/width
        offset = 0.10
        fac = max(-1.36*np.log10(width) + 3.63, 0)
        #fac = 0.3
        print(fac)
        x = target_pose.pose.pose.position.x + fac - offset #np.cos(yaw)  * fac * width
        y = target_pose.pose.pose.position.y #- np.sin(yaw)  * fac * width


        #self.publish_position(x, y, 'base_link')

        listener = tf.TransformListener()
        listener.waitForTransform("/base_link", "/odom", rospy.Time(0),rospy.Duration(4.0))
        laser_point=PointStamped()
        laser_point.header.frame_id = "base_link"
        laser_point.header.stamp =rospy.Time(0)
        laser_point.point.x=x
        laser_point.point.y=y
        laser_point.point.z=0.0
        p=listener.transformPoint("odom",laser_point)
        x = p.point.x
        y = p.point.y
        valid = fac < 1.6

        #new_vec = transform(np.asarray([x,y,0]), 'base_link', 'odom')
        #x = new_vec[0]
        #y = new_vec[1]

        self.publish_position(x, y, 'odom')


        return x, y, valid

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('face_recognition_subscriber')
    rs = RecognitionSubscriber()
    rospy.spin()
