#!/usr/bin/python

import os

import rospy

from std_msgs.msg import String, Bool
from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *


class KWSDetection(object):
    """Class to add keyword spotting functionality"""

    def __init__(self):

        # Initializing publisher with buffer size of 10 messages
        self.kws_publisher = rospy.Publisher("kws_data", String, queue_size=1)

        # initialize node
        rospy.init_node("kws_control")

        # Params
        # Params given when using launchfile because of portability on pi

	# Working with keywordslist and not grammar or language models because we saw in practice that those gave less good results
	# Those models will always want to recognise something, even if it has not been said. This would be good when working with more sentences.
	# Working with keywordlist is better when you work with a fewer commands like us
        # Directory of Hidden Markov Model
        _hmm_param = "~hmm"
        # Dictionary file containing the pronounciation of the keywords used in the command keyphrases
        _dict_param = "~dict"
        # List of commandphrases to detect with corresponding sensitivies
        _kws_param = "~kws"

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
	# setting keywordlist parameter
	self.kw_list = rospy.get_param(_kws_param)

	# other params
	# boolean saying if greeting keyphrase was detected
        self.kp_det = False
	# boolean saying if action detected
	self.action_det = False

	# Call custom function on node shutdown
        rospy.on_shutdown(self.shutdown)

        # All params satisfied. Starting recognizer
        self.start_recognizer()

    def start_recognizer(self):
        """Function to handle keyword spotting of audio"""
	# Configure decoder configuration using provided parameters
        config = Decoder.default_config()
        rospy.loginfo("Pocketsphinx initialized")
        config.set_string('-hmm', self.class_hmm)
        config.set_string('-dict', self.lexicon)
        config.set_string('-dither', "no")
        config.set_string('-featparams', os.path.join(self.class_hmm, "feat.params"))
	config.set_string('-kws', self.kw_list)

        # Set required configuration for decoder
        self.decoder = Decoder(config)

        # Start processing input audio
        self.decoder.start_utt()
        rospy.loginfo("Decoder started successfully")
        # Subscribe to audio topic
        self.audio_subscriber = rospy.Subscriber("sphinx_audio", String, self.process_audio)
	# Subscribe to topic indicating if greeting keyphrase was detected
	self.kp_det_subcriber = rospy.Subscriber("keyphrase_detection", Bool, self.kp_det_sub)
	# to callback subscribers
        rospy.spin()

    def process_audio(self, data):
        """Audio processing based on decoder config"""

        # Check if greeting keyphrase detected
        if self.kp_det:
            # Actual processing
            self.decoder.process_raw(data.data, False, False)
	    #print(self.decoder.hyp())
	    # If the decoder has an hypothesis, meaning: if the detector thinks a keyword/phrase has been said
            if self.decoder.hyp() != None:
		# showing the probability, start and end of the detected keyphrases
                rospy.loginfo([(seg.word, seg.prob, seg.start_frame, seg.end_frame)
                               for seg in self.decoder.seg()])
                rospy.loginfo("Detected action keyphrase")
		# set for conveniance everything to lowercase
                seg.word = seg.word.lower()
		# Let the decoder stop detecting utterances		
                self.decoder.end_utt()
		# publish the detected keyphrase
                self.kws_publisher.publish(seg.word)
                # self.action_det = True
                self.decoder.start_utt()

    def kp_det_sub(self, msg):
	self.kp_det = msg.data

    def shutdown():
        """This function is executed on node shutdown."""
        # command executed after Ctrl+C is pressed
        rospy.loginfo("Stop ASRControl")
        rospy.sleep(1)


if __name__ == "__main__":
    KWSDetection()
