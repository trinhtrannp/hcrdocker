#!/usr/bin/env python
import rospy
import tf
import tf_conversions
import tf2_ros
import threading

import math
import copy

import smach
import smach_ros

import geometry_msgs
import measure

__all__ = ['Logic','Reset','Score']

class Logic(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['aborted',
                                       'reset',
                                       'move_detect',
                                       'move_search',
                                       'object_grasp',
                                       'object_place'],
                             input_keys=['flg_reset',
                                         'flg_score',
                                         'flg_move_detect_done',
                                         'flg_move_search_done',
                                         'flg_object_grasp_done',
                                         'flg_object_place_done'],
                             output_keys=['pose_base_tar'])

    def execute(self, userdata):

        ### MOVE_DETECT ###
        # if False:
        if not userdata.flg_move_detect_done:
            rospy.loginfo("LOGIC ----------> MOVE_DETECT")
            return 'move_detect'

        # ### MOVE_SEARCH ###
        # if False:
        if not userdata.flg_move_search_done:
            rospy.loginfo("LOGIC ----------> MOVE_SEARCH")
            return 'move_search'

        ### OBJECT_GRASP ###
        elif not userdata.flg_object_grasp_done:
            rospy.loginfo("LOGIC ----------> OBJECT_GRASP")
            return 'object_grasp'

        ### OBJECT_PLACE ###
        elif not userdata.flg_object_place_done:
            pose =  geometry_msgs.msg.PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = 0.0
            pose.pose.position.y = 0.0
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            rospy.loginfo('LOGIC: place object at pose\n{}'.format(pose))
            userdata.pose_base_tar = copy.deepcopy(pose)
            return 'object_place'

        ### RESET ###
        rospy.loginfo("LOGIC ----------> RESET")
        return 'reset'


class Reset(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded',
                                       'aborted'],
                             input_keys=['flg_move_detect_done',
                                         'flg_move_search_done',
                                         'list_detected_frames_tmp',
                                         'list_detected_frames_1',
                                         'list_detected_frames_2',
                                         'list_detected_frames_3'],
                             output_keys=['flg_move_detect_done',
                                          'flg_move_search_done',
                                          'flg_move_new_goal',
                                          'flg_object_grasp_done',
                                          'flg_object_place_done',
                                          'list_detected_frames_tmp',
                                          'list_detected_frames_1',
                                          'list_detected_frames_2',
                                          'list_detected_frames_3'])

    def execute(self, userdata):
        userdata.flg_move_detect_done = False
        userdata.flg_move_search_done = False
        userdata.flg_object_grasp_done = False
        userdata.flg_object_place_done = False

        userdata.flg_move_new_goal = False


        del userdata.list_detected_frames_tmp[:]
        del userdata.list_detected_frames_1[:]
        del userdata.list_detected_frames_2[:]
        del userdata.list_detected_frames_3[:]

        return 'succeeded'

class Score(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded',
                                       'aborted'],
                             input_keys=['pose_base_tar','pose_object_tar',
                                         'list_detected_frames_1',
                                         'list_detected_frames_2',
                                         'list_detected_frames_3',
                                         'flg_move_detect_done',
                                         'flg_move_search_done'],
                             output_keys=['pose_base_tar','pose_object_tar'])


        self.max_distance_between_frames = 0.05 # 5cm


    def execute(self, userdata):
        list_score = []
        frame = measure.Frame()


        rospy.loginfo('SCORE: userdata.list_detected_frames_1\n')
        for frame in userdata.list_detected_frames_1:
            str_frame = str(frame)
            rospy.loginfo('{}' .format(str_frame))

        rospy.loginfo('SCORE: userdata.list_detected_frames_2\n')
        for frame in userdata.list_detected_frames_2:
            str_frame = str(frame)
            rospy.loginfo('{}' .format(str_frame))



        rospy.loginfo('SCORE: userdata.list_detected_frames_1 len: {}'
                      .format(len(userdata.list_detected_frames_1)))
        rospy.loginfo('SCORE: userdata.list_detected_frames_2 len: {}'
                      .format(len(userdata.list_detected_frames_2)))
        rospy.loginfo('SCORE: userdata.flg_move_detect_done{}'
                      .format(userdata.flg_move_detect_done))
        rospy.loginfo('SCORE: userdata.flg_move_search_done{}'
                      .format(userdata.flg_move_search_done))


        ### apend sub list (min one frame) to 'list_score'
        if userdata.flg_move_detect_done and len(userdata.list_detected_frames_1) >0:
            rospy.loginfo('SCORE: extend "list_detected_frames_1"')
            list_score.extend(userdata.list_detected_frames_1)
            run = 1

        if userdata.flg_move_search_done and len(userdata.list_detected_frames_2) >0:
            rospy.loginfo('SCORE: extend "list_detected_frames_2')
            list_score.extend(userdata.list_detected_frames_2)
            run = 2

        # if userdata.flg_move_xxx_done and len(userdata.list_detected_frames_3) >0:
        #     rospy.loginfo('SCORE: extend "list_detected_frames_3')
        #     list_score.extend(userdata.list_detected_frames_3)
        #     run = 3

        if not frame in list_score:
            rospy.loginfo("SCORE: no frame in list")
            rospy.loginfo('SCORE: ABORTED')
            return 'aborted'


        ### sorte 'list_score' by vector
        list_score = sorted(list_score, key=measure.getKey)

        ### create 'index + count' table
        scoring_table = []
        count = 0
        for idx in range(len(list_score)):

            if idx < 0:
                rospy.loginfo('SCORE: no objects in list')
                rospy.loginfo('SCORE: break')
                break

            idx_ = idx+1 if idx < len(list_score)-1 else 0

            if measure.compare_objects(list_score[idx], list_score[idx_], self.max_distance_between_frames):
                rospy.loginfo('SCORE: after compare ->  idx: {}, count: {}' .format(idx, count))
                count = count+1

             # ">=" only for case "one object in list", otherwise "==" is sufficient
            if count >= run-1:
                rospy.loginfo('SCORE: append to table idx: {}, count: {}' .format(idx, count))

                # idx is last object -1
                scoring_table.append((idx, count))
                count = 0

        rospy.loginfo('SCORE: "scor_table"\n{}' .format(scoring_table))

        if len(scoring_table)<= 0:

            rospy.loginfo("SCORE: scoring table empty")
            rospy.loginfo('SCORE: ABORTED')
            return 'aborted'


        ### use object[0] as target
        frame_tar = measure.Frame()
        frame_tar = list_score[scoring_table[0][0]]

        pose_object_tar = geometry_msgs.msg.PoseStamped()
        pose_object_tar = self.transform_pose_related_to_baseFootprint(frame_tar.get_pose())

        userdata.pose_object_tar = copy.deepcopy(pose_object_tar)

        rospy.loginfo('SCORE: "pose_object_tar" \n{}' .format(pose_object_tar))
        rospy.loginfo('SCORE: SUCCEEDED')
        return 'succeeded'


    def transform_pose_related_to_linkArmBase(self, pose):
        msg = geometry_msgs.msg.PoseStamped()
        pose_tar = geometry_msgs.msg.PoseStamped()
        msg = pose

        listener = tf.TransformListener()

        rospy.loginfo('SCORE: transforming frames in "link_armBase"')
        rospy.loginfo('SCORE: proccess ..')

        done = False
        while not done:

            try:
                pose_tar = listener.transformPose('link_armBase', msg)
                done = True

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

        rospy.loginfo('SCORE: transform done')
        return pose_tar

    def transform_pose_related_to_baseFootprint(self, pose):
        msg = geometry_msgs.msg.PoseStamped()
        pose_tar = geometry_msgs.msg.PoseStamped()
        msg = pose
        listener = tf.TransformListener()

        rospy.loginfo('SCORE: transforming frames in "baseFootprint"')
        rospy.loginfo('SCORE: proccess ..')
        done = False
        while not done:

            try:
                pose_tar = listener.transformPose('base_footprint', msg)
                done = True

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

        rospy.loginfo('SCORE: transform done')
        return pose_tar



class MoveDetectExit(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded',
                                       'aborted'],
                             input_keys=['list_detected_frames_tmp'],
                             output_keys=['list_detected_frames_1','flg_move_detect_done'])


    def execute(self, userdata):

        userdata.list_detected_frames_1 = userdata.list_detected_frames_tmp
        userdata.flg_move_detect_done = True

        return 'succeeded'



class MoveSearchExit(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded',
                                       'aborted'],
                             input_keys=['list_detected_frames_tmp'],
                             output_keys=['list_detected_frames_2','flg_move_search_done'])

    def execute(self, userdata):

        rospy.loginfo('MOVE_SEARCH_EXIT: userdata.list_detected_frames_tmp\n')
        for frame in userdata.list_detected_frames_tmp:
            str_frame = str(frame)
            rospy.loginfo('{}' .format(str_frame))



        userdata.list_detected_frames_2 = userdata.list_detected_frames_tmp
        userdata.flg_move_search_done = True

        rospy.loginfo('MOVE_SEARCH_EXIT: SUCCEEDED')
        return 'succeeded'

class BasePoseFinder(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded',
                                       'aborted'],
                             input_keys=['pose_base_tar','pose_object_tar'],
                             output_keys=['pose_base_tar'])

        self.distance_base_to_object = 0.17 # 20cm

    def execute(self, userdata):

        # project to 2D level
        pose = copy.deepcopy(userdata.pose_object_tar)
        rospy.loginfo('BASEPOSE_FINDER: pose (input) \n{}'.format(pose))



        # transform pose_objec_tar in "base_footprint"
        pose = self.transform_pose_related_to_baseFootprint(pose)
        pose.pose.position.z = 0

        # calculate vector "distance to travel"
        pose_vector = math.sqrt(pose.pose.position.x**2 + pose.pose.position.y**2)
        distance_to_travel = pose_vector - self.distance_base_to_object
        alpha_rz = math.atan2(pose.pose.position.y, pose.pose.position.x)

        pose.pose.position.x = distance_to_travel * math.cos(alpha_rz)
        pose.pose.position.y = distance_to_travel * math.sin(alpha_rz)


        q = tf.transformations.quaternion_from_euler(0,0,alpha_rz)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        rospy.loginfo('BASEPOSE_FINDER: pose (output)\n{}'.format(pose))
        userdata.pose_base_tar = copy.deepcopy(pose)


        rospy.loginfo('BASEPOSE_FINDER: SUCCEEDED')
        return 'succeeded'

    def transform_pose_related_to_baseFootprint(self, pose):
        msg = geometry_msgs.msg.PoseStamped()
        pose_tar = geometry_msgs.msg.PoseStamped()
        msg = pose

        listener = tf.TransformListener()

        rospy.loginfo('BASEPOSE_FINDER: transforming frames in "base_footprint"')
        rospy.loginfo('BASEPOSE_FINDER: proccess ..')
        done = False
        while not done:

            try:
                pose_tar = listener.transformPose('base_footprint', msg)
                done = True

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

        rospy.loginfo('BASEPOSE_FINDER: transform done')
        return pose_tar

class ObjectGraspExit(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded',
                                       'aborted'],
                             output_keys=['flg_object_grasp_done'])


    def execute(self, userdata):

        userdata.flg_object_grasp_done = True

        rospy.loginfo('OBJECT_GRASP_EXIT: SUCCEEDED')
        return 'succeeded'

class ObjectPlaceExit(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded',
                                       'aborted'],
                             output_keys=['flg_object_place_done'])


    def execute(self, userdata):

        userdata.flg_object_place_done = True

        rospy.loginfo('OBJECT_PLACE_EXIT: SUCCEEDED')
        return 'succeeded'



