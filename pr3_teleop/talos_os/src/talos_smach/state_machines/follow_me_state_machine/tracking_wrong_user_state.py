## Author: Devon Ash
## Maitnainer: noobaca2@gmail.com

import roslib
import rospy
import smach
import smach_ros

class TrackingWrongUserState(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=["TrackingCorrected", "TrackingUncorrected", "TrackingFailed", "AttemptingToRetrack"])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo("Tracking wrong user")
               
        tracking_corrected = 1

        if (tracking_corrected):
            return "TrackingCorrected"
        else:
            return "TrackingFailed"


