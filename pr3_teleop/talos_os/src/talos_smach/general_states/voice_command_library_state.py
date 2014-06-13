########################################################################
## Author: Devon Ash
## Maintainer: noobaca2@gmail.com
######################## DEVELOPER README ##############################
# Info
#
#    The voice command library takes a dictionary of
#    utterance:state_machine pairs. On hearing an utterance, 
#    the state machine is executed. When the state machine finishes
#    control returns to the initial standby state where the user must
#    access the voice command libray by first saying "OK talos" or
#    something similar.
#
########################################################################
# Imports

import roslib
import rospy
import smach
import smach_ros
from std_msgs.msg import String
from talos_speech.srv import ListenForAll

############################### CLASS DEF ##############################
class VoiceCommandLibraryState(smach.State):

    def __init__(self, utterances):

        #TODO Add the utterances and their states to the overall state 
        # machine with this one as the parent. 
        # Each one of the utterances values can 

        rospy.loginfo("##############################################")
        rospy.loginfo("#VoiceCommandLibraryState: Commands available#")
        rospy.loginfo("##############################################")
        
        outcomes = []
        # The default outcomes
        outcomes.append("WaitingForCommand")
        outcomes.append("NoCommandDetected")
        outcomes.append("CommandTimeout")

        #self.default_transitions = {}
        #self.default_transitions["WaitingForCommand"] = "VoiceCommandLibraryState"
        #self.default_transitions["CommandTimeout"] = "InitialStandbyState"

        self.utterances = utterances

        for name in self.utterances:
            # Register the commands as transitions
            #self.add_command(name, state_machine)
            # lets this state know it has the other state machine as an outcome
            outcomes.append(name)
            rospy.loginfo("# VoiceCommandLibraryState: %s", name)
        # The outcomes have to be added and this state machine has to be initialized before adding sub state machines
        #for name in self.utterances:
        #    outcomes.append(name)
        rospy.loginfo("##############################################")
        smach.State.__init__(self, outcomes)
        self.counter = 0


    # TODO CHANGE THIS SO THAT IT TRANSITIONS  TO PROPER STATE
    # Calls the service listen_for_all on the utterance keys
    # depending on which one is a match, then call that state machine
    # and when that state machine is done executing then return
    # back to main
    def execute(self, userdata):
        try:
            rospy.loginfo("Checking for utterances...")
            listen_for_all = rospy.ServiceProxy('listen_for_all', ListenForAll)

            
            words = self.utterances.keys()
            rospy.loginfo("Requesting to listen for: ")
            response = listen_for_all(words)
            
            # get the corresponding state machine
            if response is not None:
                # Transition to that state machine
                return response.result
        except rospy.ServiceException, e:
            print "Service call failed %s" %e


    # DEPRECATED Instead, the states will be added to the outer SM
    # Adds a command to the voice library
    #def add_command(self, name, state_machine):

        # Set all transitions back to the InitialStandbyState
        # All paths lead to rome/home? :D
 #       outcomes = state_machine.get_registered_outcomes()
 #       transitions = {} 
 #       for outcome in outcomes:
 #           transitions[outcome] = "InitialStandbyState"

        # instead, should register it as an outcome of this node??
 #       smach.StateMachine.add(name, state_machine, transitions)
        

