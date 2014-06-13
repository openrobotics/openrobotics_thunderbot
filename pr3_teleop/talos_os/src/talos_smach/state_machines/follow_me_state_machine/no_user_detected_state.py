## Author: Devon Ash
## Maitnainer: noobaca2@gmail.com

import roslib
import rospy
import smach
import smach_ros

class NoUserDetectedState(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=["UserDetected", "UserOffScreen", "UserOccluded", "TrackingWrongUser"])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo("No user detected")
               
        user_detected = 1

        if (user_detected):
            return "UserDetected"
        else:
            return "TrackingWrongUser"


