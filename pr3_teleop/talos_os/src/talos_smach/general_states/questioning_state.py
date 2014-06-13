########################################################################
## Author: Devon Ash
## Maintainer: noobaca2@gmail.com
######################## DEVELOPER README ##############################
# Info
#
#     The questioning state listens for a response, given a question to ask. eg. state(question)->answer. 
#
########################################################################
# Imports

import roslib; roslib.load_manifest('sound_play'); roslib.load_manifest('talos_speech')
# It doesn't work any other way, please explain someone?
import talos_speech_listener
from talos_speech_listener.speech_listener import SpeechListener


from threading import Thread
import rospy
import smach
import smach_ros

from sound_play.msg import SoundRequest
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyRequest
from talos_audio.srv import ListenFor
from talos_audio.srv import ListenForAll

from listening_state import ListeningState

############################################ CLASS DEF ##############################################
class QuestioningState(smach.State):

    def __init__(self, question):
        self.CURRENT_MODE = 0
        self.QUESTION_ASK_MODE = 1
        self.CONFIRM_ANSWER_MODE = 3
        self.AWAITING_RESPONSE_MODE = 2
        self.AWAITING_CONFIRM_RESPONSE_MODE = 4

        # Default behaviour
        self.CURRENT_MODE = self.QUESTION_ASK_MODE
        smach.State.__init__(self, outcomes=["AwaitingQuestionResponse", "ResponseReceived", "AwaitingConfirmationResponse", "ConfirmedYes", "ConfirmedNo", "WhoopsWTF"])
        self.question = question
        self.subscriber = rospy.Subscriber('recognizer/output', String, self.response_callback)
        self.counter = 0
        self.got_a_word = False


    def response_callback(self, data):
        if (self.CURRENT_MODE == self.AWAITING_RESPONSE_MODE and len(data.data) > 0):
            self.response = data.data
            self.got_a_word = True
            rospy.logdebug("GOT ME A WORD!!!!!!!!!!!!!!!!!!! %s", data.data)
            self.CURRENT_MODE = self.CONFIRM_ANSWER_MODE

    # Restores the values because things may have been modified, and the question may have to  be asked again thus start fresh. 
    def reset_questioning_variables():
        self.got_a_word = False
        self.counter = 0
        self.response = "NoResponse"
        self.CURRENT_MODE = self.QUESTION_ASK_MODE

    def execute(self, userdata):
        if (self.CURRENT_MODE == self.QUESTION_ASK_MODE):
            self.CURRENT_MODE = self.AWAITING_RESPONSE_MODE
            rospy.logdebug("MODE: QUESTION ASK MODE")
            rospy.loginfo("UNDEFINED BEHAVIOUR BUG")
            SpeechListener.say(self.question)
            rospy.loginfo("UNDEFINED BEHAVIOUR BUG")
            SpeechListener.start_recognizer()
            rospy.sleep(3)
            return "AwaitingQuestionResponse"
        
        if (self.CURRENT_MODE == self.AWAITING_RESPONSE_MODE):
            rospy.logdebug("MODE: AWAITING_RESPONSE_MODE")
            if (self.got_a_word):
                self.got_a_word = False
                return "ResponseReceived"

        if (self.CURRENT_MODE == self.CONFIRM_ANSWER_MODE):
            rospy.logdebug("MODE: CONFIRM_ANSWER_MODE")
            rospy.loginfo("BEFORE HEISENBUG")
            SpeechListener.say("Did you say " + self.response)
            rospy.sleep(3)
            self.CURRENT_MODE == self.AWAITING_CONFIRM_RESPONSE_MODE
            SpeechListener.start_recognizer()
            #self.got_a_word = False
            #self.response = "NoResponse"
            return "AwaitingConfirmationResponse"

        if (self.CURRENT_MODE == self.AWAITING_CONFIRM_RESPONSE_MODE):
            rospy.logdebug("MODE: AWAITING CONFIRM RESPONSE MODE")
            if (self.got_a_word):
                if (self.response == "yes"):
                    return "ConfirmedYes"
                if (self.response == "no"):
                    self.reset_questioning_variables()
                    return "ConfirmedNo"

        return "WhoopsWTF"


           


