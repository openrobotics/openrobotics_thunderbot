#!/usr/bin/env python

## Author: Devon Ash
## Maintainer: noobaca2@gmail.com
############################### IMPORTS ############################


import roslib; roslib.load_manifest('talos_speech')
import rospy
import pyttsx

from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyRequest
from talos_audio.srv import ListenFor
from talos_audio.srv import ListenForAll
from talos_audio.srv import ListenForAny
from sound_play.libsoundplay import SoundClient


########################### DEVELOEPR README #######################

# 1. The speech listener class works ontop of the recognizer.py from
# pocketsphinx package for ros.

# 2. Its intended use is the .listenFor()
# function which will return false until the callback from
# pocketsphinx returns the matched string. 

# 3.This is useful instead of putting a subscripter 
# in every state/class/ for the smach state machine

class TalosAudio:

    def __init__(self):
        self.engine = pyttsx.init()
        self.old_word = "No words heard"
        self.heard_word = False
        self.listening = False
        self.words_listened_for = []
        self.last_word_heard = "No words heard"
        self.listen_for_any = False

    def listen_for_words_callback(self, data):
        rospy.loginfo(rospy.get_name() + ": I heard %s", data.data)
        self.old_word = data.data

        if data.data in self.words_listened_for:
            self.heard_word = True
            rospy.loginfo(rospy.get_name() + ": Word '%s' has been heard", data.data)
            self.last_word_heard = data.data
            self.stop_listening()
        elif self.listen_for_any:
            self.last_word_heard = data.data
            self.stop_listening()

    def listen_for_all(self, request):
        if not self.listening and not self.heard_word:
            self.words_listened_for = request.words
            self.start_listening()
            rospy.loginfo("Listening for an array of phrases")
        else:
            if (self.heard_word):
                self.words_listened_for = []
                self.heard_word = False
                self.listening = False
                return self.last_word_heard
            
        return "NoCommandDetected"

    def stop_listening(self):
        try:
                # Once the words have been heard, no need to continue listening, shut down the listening. 
            stop = rospy.ServiceProxy('recognizer/stop', Empty)
            response = stop()
        except rospy.ServiceException, e:
            print "Service call failed %s" %e
            
        self.listening = False

    # Tells recognizer/output to start producing values it hears
    def start_listening(self):
        try:
            # Start listening to the recognizer callbacks
            start = rospy.ServiceProxy('recognizer/start', Empty)
            response = start()
        except rospy.ServiceException, e:
            print "Service call failed %s" %e

        # At the end of the function, because if the service call fails
        # it is not good to have this set to true
        self.heard_words = False
        self.listening = True

    @staticmethod
    def start_recognizer():
        try:
            # Start listening to the recognizer callbacks
            start = rospy.ServiceProxy('recognizer/start', Empty)
            response = start()
        except rospy.ServiceException, e:
            print "Service call failed %s" %e

    @staticmethod
    def stop_recognizer():
        try:
            # Start listening to the recognizer callbacks
            start = rospy.ServiceProxy('recognizer/stop', Empty)
            response = start()
        except rospy.ServiceException, e:
            print "Service call failed %s" %e
 
    def say(self, utterance):
        self.engine.say(utterance)
        self.engine.runAndWait()

def main():

    listener = SpeechListener()
    rospy.init_node("speech_listener")
    rospy.loginfo(rospy.get_name() + ": Started speech listener")
    rospy.Subscriber("recognizer/output", String, listener.listen_for_words_callback)
    listen_for_all_service = rospy.Service('listen_for_all', ListenForAll, listener.listen_for_all)
   
    say_service = rospy.Service('say', ListenFor, listener.say)

    try:
        # On startup, do not listen for anything
        listener.stop_listening()
        rospy.loginfo("Stopping recognizer/output service. This node will activate it again when asked to listen for something.")
    except rospy.ServiceException, e:
        print "Service call failed %s" %e
        
    rospy.spin()
    
if __name__ == "__main__":
    main()
