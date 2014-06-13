########################################################################
## Author: Devon Ash
## Maintainer: noobaca2@gmail.com
######################## DEVELOPER README ##############################
# Info
#
#     The follow me algorithm used in the RoboCup@Home competition for
#     Thunderbots@Home team coded as a state machine
#
########################################################################
# Imports

import roslib
import rospy
import smach
import smach_ros

# General state imports
from talos_smach.general_states.listening_state import ListeningState
from talos_smach.general_states.questioning_state import QuestioningState

# Follower state imports
from talos_smach.state_machines.follow_me_state_machine.occluded_state import OccludedState
from talos_smach.state_machines.follow_me_state_machine.positioning_for_user_state import PositioningForUserState
from talos_smach.state_machines.follow_me_state_machine.remembering_user_state import RememberingUserState
from talos_smach.state_machines.follow_me_state_machine.user_off_screen_state import UserOffScreenState
from talos_smach.state_machines.follow_me_state_machine.no_user_detected_state import NoUserDetectedState
from talos_smach.state_machines.follow_me_state_machine.following_state import FollowingState
from talos_smach.state_machines.follow_me_state_machine.tracking_unidentified_state import TrackingUnidentifiedUserState
from talos_smach.state_machines.follow_me_state_machine.tracking_wrong_user_state import TrackingWrongUserState
from talos_smach.state_machines.follow_me_state_machine.failed_tracking_state import FailedTrackingState

############################### CLASS DEF ##############################
class FollowMeStateMachine(smach.StateMachine):

    ## When done, it should return the state "InitialStandbyState"
    def __init__(self):
        outcomes = []
        outcomes.append("Succeeded")
        outcomes.append("Failed")
        super(FollowMeStateMachine, self).__init__(outcomes)

        #TODO: abstract the states for the generic detection, recognition, and speech states. 
        with self:

            # LISTEN FOR REMEMBER ME STATE
            transitions = {}
            transitions["CommandDetected"] = "AskTheirNameState"
            transitions["NoCommandDetected"] = "RememberMeStandbyState"
            smach.StateMachine.add("RememberMeStandbyState", ListeningState("remember"), transitions)

            # ASK FOR THEIR NAME STATE
            questioning_state = QuestioningState("What is your name?")
            transitions = {}
            transitions["AwaitingQuestionResponse"] = "AskTheirNameState"
            transitions["AwaitingConfirmationResponse"] = "AskTheirNameState"
            transitions["ResponseReceived"] = "AskTheirNameState"
            transitions["ConfirmedYes"] = "TrackingUnidentifiedUserState"
            transitions["ConfirmedNo"] = "AskTheirNameState"
            transitions["WhoopsWTF"] = "AskTheirNameState"
            smach.StateMachine.add("AskTheirNameState", questioning_state, transitions)

            # TRACKING UNIDENTIFIED USER STATE
            smach.StateMachine.add('TrackingUnidentifiedUserState', TrackingUnidentifiedUserState(), transitions={'TrackingFailed':'FailedTrackingState', 'IdentifiedUser':'RememberingUserState', 'ContinueTracking':'TrackingUnidentifiedUserState'})

            # REMEMBER ME STATE	
            smach.StateMachine.add('RememberingUserState', RememberingUserState(),transitions={"UserRemembered":'FollowingCommandStandbyState',"UnsuccessfulAttempt":'RememberingUserState'})
            # FOLLOWING STANDBY STATE
            transitions = {}
            transitions["CommandDetected"] = "FollowingState"
            transitions["NoCommandDetected"] = "FollowingCommandStandbyState"
            smach.StateMachine.add('FollowingCommandStandbyState', ListeningState('follow'), transitions)

            # FOLLOWING STATE
            smach.StateMachine.add("FollowingState", FollowingState(), transitions={"ContinueFollowing":"FollowingState", "NoUserDetected":"NoUserDetectedState"})

            # NO USER DETECTED STATE
            smach.StateMachine.add("NoUserDetectedState", NoUserDetectedState(), transitions={"UserDetected":"FollowingState", "UserOffScreen":"UserOffScreenState", "UserOccluded":"OccludedState", "TrackingWrongUser":"TrackingWrongUserState"})

            # FAILED TRACKING STATE
            smach.StateMachine.add("FailedTrackingState", FailedTrackingState(), transitions={"FailedTracking":"Failed"})

            # TRACKING WRONG USER STATE
            smach.StateMachine.add("TrackingWrongUserState", TrackingWrongUserState(), transitions={"TrackingCorrected":"FollowingState", "TrackingUncorrected":"TrackingWrongUserState", "TrackingFailed":"FailedTrackingState", "AttemptingToRetrack":"TrackingWrongUserState"})

            # OCCLUDED STATE
            smach.StateMachine.add("OccludedState", OccludedState(), transitions={"WaitingForUser":"OccludedState", "PositioningForUser":"PositioningForUserState", "FailedToFindUser":"FailedTrackingState"})

            # POSITIONING FOR USER STATE
            smach.StateMachine.add("PositioningForUserState", PositioningForUserState(), transitions={"UserDetected":"FollowingState", "PositioningForUser":"PositioningForUserState", "PositioningForUserFailed":"FailedTrackingState"})

            # USER OFF SCREEN STATE
            smach.StateMachine.add("UserOffScreenState", UserOffScreenState(), transitions={"UserDetected":"FollowingState", "UserOffScreenStill":"UserOffScreenState", "FailedToFindUser":"FailedTrackingState"})


