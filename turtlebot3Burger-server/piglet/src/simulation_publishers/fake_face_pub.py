#!/usr/bin/python
import rospy
from std_msgs.msg import MultiArrayLayout, Float32MultiArray, MultiArrayDimension

len_faces = 4

def create_array_message():
	array_dim_0 = MultiArrayDimension()
	array_dim_0.label = 'dim_0'
	array_dim_0.size = 4
	array_dim_0.stride = 0
	array_dim_1 = MultiArrayDimension()
	array_dim_1.label = 'dim_1'
	array_dim_1.size = len_faces
	array_dim_1.stride = 0
	array_layout = MultiArrayDimension()
	array_layout.label = "face_array"
	array_message = Float32MultiArray()
	array_message.layout.dim = [array_dim_0, array_dim_1]
	array_message.layout.data_offset = 0
	face_0 = [0.0, 1.0, 1.0, 200.0]
	face_1 = [1.0, 0.0, 1.0, 100.0]
	face_2 = [2.0, 1.0, 0.0, 300.0]
	face_3 = [3.0, 0.0, 0.0, 400.0]
	array_message.data = face_0 + face_1 + face_2 + face_3
	return array_message

if __name__ == '__main__':
	rospy.init_node('fake_face_publisher')


	face_pub = rospy.Publisher('/custom/people_simple', Float32MultiArray, queue_size=5)
	faces_message = create_array_message()

	while not rospy.is_shutdown():
		face_pub.publish(faces_message)