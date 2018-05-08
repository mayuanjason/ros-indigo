#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient


class TalkBack:

    def __init__(self, script_path):
        rospy.init_node('talkback')

        rospy.on_shutdown(self.cleanup)

        # Set the default TTS voice to use
        self.voice = rospy.get_param("~voice", "voice_don_diphone")

        # Set the wave file path if used
        self.wavepath = rospy.get_param("~wavepath", script_path + "/.. / sounds")

        # Create the sound client object
        self.soundhandle = SoundClient()

        # wait a moment to let the client connect to the sound_play server
        rospy.sleep(1)

        # Make sure any lingering sound_play processes are stoped.
        self.soundhandle.stopAll()

        # Announce that we are ready for input
        self.soundhandle.playWave(self.wavepath + "R2D2a.wav")
        rospy.sleep(1)
        self.soundhandle.say("Ready", self.voice)

        rospy.loginfo("Say one of the navigation commmands...")

        # Subscribe to the recognizer output and set the callback function
        rospy.Subscriber('/recognizer/output', String, self.talkback)

    def talkback(self, msg):
        # Print the recognizerd words on the screen
        rospy.loginfo(msg.data)

        # Speak the recognized words in the selected voice
        self.soundhandle.say(msg.data, self.voice)

    def cleanup(self):
        self.soundhandle.stopAll()
        rospy.loginfo("Shutting down talkback node...")


if __name__ == '__main__':
    try:
        TalkBack(sys.path[0])
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Talkback node terminated.")
