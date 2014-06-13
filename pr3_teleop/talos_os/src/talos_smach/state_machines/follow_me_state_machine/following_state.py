## Author: Devon Ash
## Maitnainer: noobaca2@gmail.com

import roslib
import rospy
import smach
import smach_ros

class FollowingState(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=["ContinueFollowing", "NoUserDetected"])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo("Tracking user")
               
        continue_following = 1

        if (continue_following):
            return "ContinueFollowing"
        else:
            return "FollowingFailed"


