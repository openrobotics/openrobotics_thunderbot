## Author: Devon Ash
## Maitnainer: noobaca2@gmail.com

import roslib
import rospy
import smach
import smach_ros

class TrackingUnidentifiedUserState(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=["TrackingFailed", "IdentifiedUser", "ContinueTracking"])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo("Tracking user")
               
        continue_tracking = 1

        if (continue_tracking):
            return "ContinueTracking"
        else:
            return "IdentifiedUser"


