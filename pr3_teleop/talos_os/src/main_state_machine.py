#!/usr/bin/env python
## Author: Devon Ash
## Maintainer: noobaca2@gmail.com
########################### Import Libraries #######################

import roslib
import rospy
import smach
import smach_ros


############################ Import states #########################
# DEPRECATED
#from talos_smach.command_standby_state import CommandStandbyState
#from talos_smach.failed_state_prompt import FailedStatePrompt
#from talos_smach.follower_command_standby_state import FollowerCommandStandbyState
#from talos_smach.remembering_user_state import RememberingUserState
#from talos_smach.startup_state import StartupState
#from talos_smach.tracking_unidentified_state import TrackingUnidentifiedUserState

# DEPRECATED
#Follower algorithm includes
#from talos_smach.follow_states.failed_tracking_state import FailedTrackingState
#from talos_smach.follow_states.occluded_state import OccludedState
#from talos_smach.follow_states.tracking_wrong_user_state import TrackingWrongUserState
#from talos_smach.follow_states.user_off_screen_state import UserOffScreenState
#from talos_smach.follow_states.no_user_detected_state import NoUserDetectedState
#from talos_smach.follow_states.following_state import FollowingState
#from talos_smach.follow_states.positioning_for_user_state import PositioningForUserState

# General state includes
from talos_smach.general_states.listening_state import ListeningState
from talos_smach.general_states.voice_command_library_state import VoiceCommandLibraryState
from talos_smach.state_machines.follow_me_state_machine.follow_me_state_machine import FollowMeStateMachine
################################ MAIN ##############################


class MainStateMachine:
    
    def __init__(self):
        self.sm = smach.StateMachine(outcomes=['success', 'failure'])

    # Adds the initial startup states to talos
    def add_startup_states(self):

        with self.sm:

            # STARTUP STATE
            #smach.StateMachine.add('InitialStandbyState', ListeningState("ok"),transitions={"NoCommandDetected":'InitialStandbyState', "CommandDetected":'VoiceCommandLibraryState'})

            utterances = {}
            utterances["follow"] = FollowMeStateMachine()

            voice_command_lib = VoiceCommandLibraryState(utterances)
            transitions = {}
            transitions["WaitingForCommand"] = "VoiceCommandLibraryState"
            transitions["NoCommandDetected"] = "VoiceCommandLibraryState"
            transitions["CommandTimeout"] = "VoiceCommandLibraryState"
            transitions["follow"] = "FollowMeStateMachine"

            # Adding the voice commands to the library            
            smach.StateMachine.add('VoiceCommandLibraryState', voice_command_lib, transitions)

            # Add the state machines that it transitions to
            transitions = {}
            transitions["Succeeded"] = "VoiceCommandLibraryState"
            transitions["Failed"] = "VoiceCommandLibraryState"
            smach.StateMachine.add('FollowMeStateMachine', FollowMeStateMachine(), transitions)


def main():
    rospy.init_node('talos_main_state_machine')
    
    #Create a Smach
    sm = MainStateMachine()
    sm.add_startup_states()
    
    sis = smach_ros.IntrospectionServer('server_name', sm.sm, '/SM_ROOT')
    sis.start()
    #sm.add_follow_me_states()
    outcome = sm.sm.execute()

########################### Start Main #############################

if __name__ == '__main__':
    main()
