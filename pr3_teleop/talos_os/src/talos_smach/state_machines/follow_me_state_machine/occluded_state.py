## Author: Devon Ash
## Maitnainer: noobaca2@gmail.com

import roslib
import rospy
import smach
import smach_ros

class OccludedState(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=["WaitingForUser", "PositioningForUser", "FailedToFindUser"])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo("Tracking user")
               
        waiting_for_user = 1

        if (waiting_for_user):
            return "WaitingForUser"
        else:
            return "FailedToFindUser"


