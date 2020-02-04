#!/usr/bin/env python

import rospy
import smach
import smach_ros

import copy

import move_base_msgs.msg
import geometry_msgs.msg
import actionlib
from actionlib_msgs.msg import GoalStatus

__all__ = ['Move','Stop']

class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded',
                                       'aborted'],
                             input_keys=['pose_base_tar'],
                             output_keys=['pose_base_cur'])

        self.feedback = move_base_msgs.msg.MoveBaseFeedback()

        self.timeout = 100 #sec

        self.client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
        rospy.loginfo('MOVE: Connecting to move_base...')
        self.client.wait_for_server()
        rospy.loginfo('MOVE: Connected to move_base.')

    def move_base_feedback_cb(self, feedback):
        self.feedback = feedback

    def execute(self, userdata):
        pose_base_tar = geometry_msgs.msg.PoseStamped()
        pose_base_tar = copy.deepcopy(userdata.pose_base_tar)

        goal = move_base_msgs.msg.MoveBaseGoal()
        goal.target_pose.header.frame_id = pose_base_tar.header.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose_base_tar.pose


        rospy.loginfo("MOVE: send goal: \n{}" .format(goal))
        self.client.send_goal(goal, feedback_cb=self.move_base_feedback_cb)
        success = self.client.wait_for_result(rospy.Duration(self.timeout))

        if success:
            state = self.client.get_state()
            userdata.pose_base_cur = copy.deepcopy(self.feedback.base_position)

            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("MOVE: SUCCEEDED")
                return 'succeeded'

            elif state == GoalStatus.ABORTED:
                rospy.loginfo("MOVE: move_base stauts: aborted")
                rospy.loginfo("MOVE: ABORTED")
                self.client.cancel_goal()
                return 'succeeded'
                # return 'aborted'

            else:
                rospy.loginfo("MOVE: move_base status is not succeeded or aborted")

        else:
            rospy.loginfo("MOVE: Timeout while moveing")
            self.client.cancel_goal()
            rospy.loginfo("MOVE: cancel all goals")

        rospy.loginfo("MOVE: ABORTED")
        return 'succeeded'
        # return 'aborted'


class Stop(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded',
                                       'aborted'],
                             output_keys=['pose_base_cur'])

        self.timeout = 60 #sec
        self.move_base_feedback = move_base_msgs.msg.MoveBaseActionFeedback()

        self.client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')


        def move_base_feedback_cb(feedback):
            self.move_base_feedback = feedback

        rospy.Subscriber("/move_base/feedback", move_base_msgs.msg.MoveBaseFeedback, move_base_feedback_cb)


    def execute(self, userdata):

        self.client.cancel_all_goals()

        # check for request, 'wait_for_result' don't work!
        # success = self.client.wait_for_result(rospy.Duration(self.timeout))
        try:
            userdata.pose_base_cur = self.move_base_feedback.feedback.base_position
        except:
            rospy.loginfo("STOP: exception!")
            rospy.loginfo("STOP: ABORTED")
            return 'aborted'

        rospy.loginfo("STOP: SUCCEEDED")
        return 'succeeded'


