## Author: Devon Ash
## Maitnainer: noobaca2@gmail.com

import roslib
import rospy
import smach
import smach_ros

class UserOffScreenState(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=["UserDetected", "UserOffScreenStill", "FailedToFindUser"])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo("User Off Screen State")

        user_detected = 1

        if (user_detected):
            return "UserDetected"
        else:
            return "UserOffScreenStill"


