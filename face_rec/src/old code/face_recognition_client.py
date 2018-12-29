#!/usr/bin/python
import roslib; roslib.load_manifest('face_rec')
import rospy

from face_rec.srv import DetectFace
import numpy as np 
from cv_bridge import CvBridge, CvBridgeError
import sys

import picamera
import picamera.array 
import time

def get_image():
    with picamera.PiCamera() as picam:
        picam.start_preview()
        time.sleep(2)
        with picamera.array.PiRGBArray(picam) as stream:
            picam.capture(stream, format='rgb')
            return stream.array

if __name__ == '__main__':
    rospy.init_node('face_recognition_client')
    rospy.wait_for_service('face_rec')

    bridge = CvBridge()
    recognizer = rospy.ServiceProxy('face_rec', DetectFace)
    image = get_image()

    frame = bridge.cv2_to_imgmsg(image, "bgr8")
    res = recognizer(frame)
    print('message received') 
    print(res)
    print("+"*100)
