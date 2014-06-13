## Author: Devon Ash
## Maitnainer: noobaca2@gmail.com

import roslib
import rospy
import smach
import smach_ros

class PositioningForUserState(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=["UserDetected", "PositioningForUser", "PositioningForUserFailed"])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo("Re-Positioning To find user")
               
        continue_tracking = 1

        return "UserDetected"


