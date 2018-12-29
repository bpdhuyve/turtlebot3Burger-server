#!/usr/bin/python
'''
Service for face recognition:
- input
    - image
- output
    - person ids
    - person locations in frame
'''
import roslib; roslib.load_manifest('face_rec')
import rospy
from face_rec.srv import DetectFace, DetectFaceResponse
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError
from FaceRecognizer import FaceRecognizer
from geometry_msgs.msg import Point
import numpy as np
import copy 
from std_msgs.msg import Float32MultiArray, MultiArrayLayout

class  RecognitionServer(object):
    '''
    In class so that the same face recognizer class is used
    for all images (Ids and face embeddings are stored there)
    # -- 
    When receiving a request, it publishes a simple solution on '/custom/people_simple' 
    '''
    def __init__(self):
        self.fr = FaceRecognizer()
        self.bridge = CvBridge()
        self.odom_sub = rospy.Subscriber(
            '/odom', Pose, self.__odom_callback) 
        self.odom = Pose()

        self.recognition_pub = rospy.Publisher(
            '/custom/people_simple', Float32MultiArray, queue_size=1) 

    def __odom_callback(self, msg): 
        self.odom = msg

    def __get_center(self, location):
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

    def __calculate_people_positions(self):
        _,_,yaw = tf.transformations.euler_from_quaternion([
            self.odom.pose.orientation.x,
            self.odom.pose.orientation.y,
            self.odom.pose.orientation.w,
            self.odom.pose.orientation.z,
        ])
        target_pose = copy.deepcopy(self.odom)
        target_pose.position.x += np.sin(yaw) 
        target_pose.position.x += np.cos(yaw) 
        return target_pose.x, target_pose.y
        
    def handle_service_request(self, request):
        cv_image = self.bridge.imgmsg_to_cv2(request.img, 'bgr8')
        ret = self.fr.recognize_people(cv_image)
        if ret: 
            locs, ids = ret
            # Publish simple info on the simple_people topic 
            simple_msg = Float32MultiArray() 
            dim = MultiArrayDimension()
            dim.label = 'id_and_pos'
            dim.size = 3
            dim.stride = 0 
            dim2 = MultiArrayDimension()
            dim2.label = 'length_faces'
            dim2.size = len(ids)
            dim2.stride = 0 
            simple_msg.layout.dim.label = [dim, dim2]
            simple_msg.data_offset = 0
            data = []
            for i in ids:
                x, y = self.__calculate_people_positions() 
                np.concatenate(data, [i,x,y]) 
            simple_msg.data = data

            print(len(data), 3 * len(ids)) 
            print(simple_msg) 

            # publish the simple message 
            self.recognition_pub(simple_msg)

            # Handle service request 
            message = DetectFaceResponse()
            message.ids = ids
            message.positions = [self.__get_center(loc) for loc in locs]
            return message
        else:
            return DetectFaceResponse()

if __name__ == '__main__':
    rospy.init_node('face_recognition_server')
    rs = RecognitionServer()
    service = rospy.Service(
        'face_rec', DetectFace, rs.handle_service_request)
    rospy.spin()
