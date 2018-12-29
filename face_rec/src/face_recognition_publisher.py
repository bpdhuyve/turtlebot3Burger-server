#!/usr/bin/python
import roslib; roslib.load_manifest('face_rec')
import rospy

import numpy as np 
from cv_bridge import CvBridge, CvBridgeError
import sys

import picamera
import picamera.array 
import time

from sensor_msgs.msg import Image

def get_image():
    '''
    Read image from the picamera and returns it as a numpy array
    '''
    with picamera.PiCamera() as picam:
        picam.start_preview()
        time.sleep(2)
        with picamera.array.PiRGBArray(picam) as stream:
            picam.capture(stream, format='rgb')
            return stream.array

if __name__ == '__main__':
    # Node that sends images from the turtlebot to the compute_faces_and_positions topic 
    rospy.init_node('face_recognition_publisher')

    compute_pub = rospy.Publisher('/custom/compute_faces_and_positions', Image, queue_size=1)

    # send images to the computation node 
    rate = rospy.Rate(4) 
    while not rospy.is_shutdown():
        bridge = CvBridge()
        frame = bridge.cv2_to_imgmsg(get_image(), "bgr8")
        compute_pub.publish(frame)
        #rate.sleep()
