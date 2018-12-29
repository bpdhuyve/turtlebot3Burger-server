#!/usr/bin/python

import os

import rospy
import pyttsx

from std_msgs.msg import String, Bool
from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
from time import time

class KPDetection(object):
    """Class to add keyphrase spotting functionality"""

    def __init__(self):
        # Initalizing publishers
	# to publish to indicate keyphrase has been detected
        self.keyphrase_publisher = rospy.Publisher("keyphrase_detection", Bool, queue_size=1)
	# to publish at end of command to errase command
	self.kws_publisher = rospy.Publisher("kws_data", String, queue_size=1)

        # initialize node
        rospy.init_node("kp_control")

	# Params given when using launchfile because of portability on pi
        # Directory of Hidden Markov Model
        _hmm_param = "~hmm"
        # Dictionary file containing the pronounciation of the keywords used in the keyphrase
        _dict_param = "~dict"

	# Setting param values
	# Check if the model is installed in the right folder
        if os.path.isdir(rospy.get_param(_hmm_param)):
            rospy.loginfo("Loading the model")
            self.class_hmm = rospy.get_param(_hmm_param)
            rospy.loginfo("Done loading the model")
        else:
            rospy.logerr(
                "Couldn't find model. Please check if installed in human_robot_interaction/model")
            return
	# setting dictionary parameter
        self.lexicon = rospy.get_param(_dict_param)
	# the used keyphrase to start listening for commands
        self.keyphrase = "hi piglet"
	# threshold to sense the keyphrase really being said
        self.kws_threshold = 1e-10

	# other params
	# boolean saying if keyphrase was detected
	self.kp_det = False
	# time stamp to remember robot when been greeted, see further
	self.kp_time = 0
	# how long the robot remembers being greeted
	self.wait_time = 5

        # Call custom function on node shutdown
        rospy.on_shutdown(self.shutdown)
	
	# All params satisfied. Starting recognizer
        self.kp_recognizer()

    def kp_recognizer(self):
        """Function to handle keyphrase spotting of audio"""
	# Configure decoder configuration using provided parameters
        config = Decoder.default_config()
        rospy.loginfo("Pocketsphinx initialized")
        config.set_string('-hmm', self.class_hmm)
        config.set_string('-dict', self.lexicon)
        config.set_string('-dither', "no")
        config.set_string('-featparams', os.path.join(self.class_hmm, "feat.params"))
        config.set_string('-keyphrase', self.keyphrase)
        config.set_float('-kws_threshold', self.kws_threshold)

        # Set required configuration for decoder
        self.decoder = Decoder(config)

        # Start processing input audio
        self.decoder.start_utt()
        rospy.loginfo("Decoder started successfully")
	# subscribe to the audio topic
        self.audio_subscriber = rospy.Subscriber("sphinx_audio", String, self.proces_audio, queue_size=10)
	# to callback subscriber 
	rospy.spin()

    def proces_audio(self, data):
        """Audio processing based on decoder config"""

        # Check if keyphrase detected
        if not self.kp_det:
            # Actual processing
            self.decoder.process_raw(data.data, False, False)

            if self.decoder.hyp() != None:
		# showing the probability, start and end of the detected keyphrase
                rospy.loginfo([(seg.word, seg.prob, seg.start_frame, seg.end_frame)
                               for seg in self.decoder.seg()])
                rospy.loginfo("Detected keyphrase")
		# Let the decoder stop detecting utterances
		self.decoder.end_utt()

		# First let piglet greet user when greeted
		message = "hi smartypants"
		# Initialise text to speech engine
		engine = pyttsx.init()
		# Define which voice to be used: english+f5 is the best voice
		engine.setProperty('voice', 'english+f5')
		# Define how fast robot talks in words/min
		engine.setProperty('rate', 160)
		# Let the robot say the message
		engine.say(message)
		engine.runAndWait()

		# Then enable the bool, because if you change these two in order, piglet would hear itself
                keyphrase_bool = Bool()
		keyphrase_bool.data = True;
		# Publish the boolean indicating if keyphrase has been detected
                self.keyphrase_publisher.publish(keyphrase_bool)
		self.kp_det = True
		self.kp_time = time()
	# if keyphrase detected
	elif self.kp_det:
	    # then after a waitperiod
	    if(time() - self.kp_time > self.wait_time):
		# let it forget it heard the keyphrase by publishing this boolean as False
		keyphrase_bool = Bool()
		keyphrase_bool.data = False
                self.keyphrase_publisher.publish(keyphrase_bool)
		self.kp_det = False
		rospy.loginfo("Forgot hearing keyphrase")
		# clean up command action because this session has ended
		kws_string = String()
		kws_string.data = ""
		self.kws_publisher.publish(kws_string)
		# start listening to utterance
                self.decoder.start_utt()
	# just printing for debugging reasons
	# print(str(time()-self.kp_time), str(self.kp_det), str(self.kp_time))

    def shutdown():
        """This function is executed on node shutdown."""
        # command executed after Ctrl+C is pressed
        rospy.loginfo("Stop kws_control")
        rospy.sleep(1)

if __name__ == "__main__":
    KPDetection()
