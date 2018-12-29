#!/usr/bin/python

from time import sleep

import pyaudio

import rospy

from std_msgs.msg import String


class AudioMessage(object):
    """Class to publish audio to topic"""

    def __init__(self):

        # Initializing publisher with buffer size of 10 messages
        self.audio_publisher = rospy.Publisher("sphinx_audio", String, queue_size=1)

        # initialize node
        rospy.init_node("audio_control")
        # Call custom function on node shutdown
        rospy.on_shutdown(self.shutdown)

        # All set, publish to topic
        self.audio_pub()

    def audio_pub(self):
        """Function to publish input audio to topic"""

        rospy.loginfo("audio input node will start after delay of 5 seconds")
        sleep(5)

        # Initializing pyaudio for input from system microphone
        stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=1024)
        stream.start_stream()


        while not rospy.is_shutdown():
            buf = stream.read(1024)
            if buf:
                # Publish audio to topic
                self.audio_publisher.publish(buf)
            else:
                rospy.loginfo("Buffer returned null")
                break

    def shutdown():
        """This function is executed on node shutdown."""
        # command executed after Ctrl+C is pressed
        rospy.loginfo("Stop audio_control")
        rospy.sleep(1)


if __name__ == "__main__":
    AudioMessage()
