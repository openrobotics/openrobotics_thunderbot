## Author: Devon Ash
## Maitnainer: noobaca2@gmail.com

import roslib
import rospy
import smach
import smach_ros

class FailedTrackingState(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=["FailedTracking"])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo("Failed Tracking")
               
        continue_tracking = 1

        return "Failed Tracking"


