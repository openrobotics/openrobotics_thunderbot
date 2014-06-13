########################################################################
## Author: Devon Ash
## Maintainer: noobaca2@gmail.com
########################### DEVELOPER README ###########################
# Info:
#    
#    The listening state is an abstraction that puts the robot into a #    pause until it has heard the phrase or utterance in the parameter #    to the class. Once heard, it will move forward onto the next state #    and before that, execute some given function. 
#
########################################################################
# Imports

import roslib
import rospy
import smach
import smach_ros
from std_msgs.msg import String
from talos_audio.srv import ListenForAll

############################# CLASS DEF ################################
class ListeningState(smach.State):

    def __init__(self, utterance):
        self.utterance = utterance
        smach.State.__init__(self, outcomes=["CommandDetected", "NoCommandDetected"])
        self.counter = 0
        rospy.loginfo("ListeningState listening for: %s", self.utterance)
        self.word_heard = "NoWordHeard"

    def execute(self, userdata):
        rospy.wait_for_service('listen_for_all')
        
        try:
            rospy.loginfo("Waiting for utterance: %s", self.utterance)
            listen_for_all = rospy.ServiceProxy('listen_for_all', ListenForAll)

            words = []
            
            if (isinstance(self.utterance, list)):
                words = self.utterance
            elif (isinstance(self.utterance, str)):
                words.append(self.utterance)
    
            response = listen_for_all(words)
            self.word_heard = response.result            

            if (isinstance(self.utterance, str)):
                if (response.result == self.utterance):
                    return "CommandDetected"
            elif (isinstance(self.utterance, list)):
                if (response.result in list):
                    return "CommandDetected"

        except rospy.ServiceException, e:
            print "Service call failed: %s" %e

        return "NoCommandDetected"




