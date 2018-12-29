#!/usr/bin/python

import rospy

from std_msgs.msg import String, Int16
from time import time
import pyttsx

class action_converter(object):
	def __init__(self):
	    # initialise node
	    rospy.init_node('action_handler')

	    # initialise params
	    self.kws_string = ""
	    # Call custom function on node shutdown
	    rospy.on_shutdown(self.shutdown)
	    # Create publisher
	    self.action_integer_publisher = rospy.Publisher('action_integer', Int16, queue_size=1)
	    # Create subscribers
	    self.kws_subscriber = rospy.Subscriber("kws_data", String, self.kws_sub, queue_size=1)
	    
	    # other params
	    # to store the previous command
	    self.old_string = ""
	    # store time of last response to command
	    self.last_time = 0
	    rospy.spin()

	def kws_sub(self, msg):
	    # Check what the user said and handle which corresponding predefined action to do.
	    # store the said command
	    self.kws_string = msg.data
	    # compare the lowercase string
	    data_lower = self.kws_string.lower()
	    # only do something when new command came in, or if same command said again after some time
    	    if (data_lower != self.old_string) or \
	       ((data_lower == self.old_string) and (time() - self.last_time) > 3):
		    # Remember the recognised command
		    self.old_string = data_lower
		    # Portability to test other things. So an integer means an predefined action independently how the used said it
		    action_int = Int16()
		    # If said action recognised map to integer, define speech rate and define message
		    if(data_lower == "stop"):
			action_int.data = 0
			speech_rate = 160
			message = "stopping"
		    elif(data_lower == "start"):
			action_int.data = 1
			speech_rate = 160
			message = "starting"
		    elif(data_lower == "move forward"):
			action_int.data = 2
			speech_rate = 160
			message = "moving forward"
		    elif(data_lower == "move back"):
			action_int.data = 3
			speech_rate = 160
			message = "moving back"	
		    elif(data_lower == "turn left"):
			action_int.data = 4
			speech_rate = 160
			message = "turning left"
		    elif(data_lower == "turn right"):
			action_int.data = 5
			speech_rate = 160
			message = "turning right"
		    elif(data_lower == "bring me a smarty"):
			action_int.data = 6
			speech_rate = 200
			message = "I bring bring bring bring bring bring bring smarties on this table the whole day through. To bring bring bring bring bring bring bring smarties is what I really like to do."
		    else:
			message = False
			action_int.data = -1
		    # Publish the action integer
		    self.action_integer_publisher.publish(action_int)
    		    # Logging what user said
		    if action_int.data >= 0:
			rospy.loginfo("User said: "+ data_lower + " with corresponding integer " + str(action_int.data))
		    # Robot responding to user when defined action recognised
		    if message:
			# Add oink after each message, because it's a piglet
			message = message + ". Oink."
			# Logging what robot said
			rospy.loginfo("Robot said: " + message)
			# initialise the text to speech engine
			engine = pyttsx.init()
			# english+f5 is the best voice
			engine.setProperty('voice', 'english+f5')
			# define speech rate = how fast the robot speaks
			engine.setProperty('rate', speech_rate)
			# let the robot say the message
			engine.say(message)
			engine.runAndWait()
			# remember when robot 
			self.last_time = time()

	def shutdown():
            """This function is executed on node shutdown."""
            # command executed after Ctrl+C is pressed
            rospy.loginfo("Stop speech action handler")
            rospy.sleep(1)

if __name__ == "__main__":
	action_converter()
	
