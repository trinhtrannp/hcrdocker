#!/usr/bin/env python
import rospy
import tf

import re           #regular exeption
import math

import smach
import smach_ros

import actionlib
from actionlib_msgs.msg import *

from geometry_msgs.msg import PoseStamped, TransformStamped
from object_recognition_msgs.msg import RecognizedObjectArray

__all__ = ['Detect','Filter']

def getKey(frame):
    return frame.absolut_vector

def compare_objects(obj1, obj2, max_distance):

    frame1 = obj1.get_pose()
    frame2 = obj2.get_pose()

    dx = frame1.pose.position.x - frame2.pose.position.x
    dy = frame1.pose.position.y - frame2.pose.position.y
    dz = frame1.pose.position.z - frame2.pose.position.z

    distance = math.sqrt(dx**2 + dy**2 + dz**2)

    if distance <= max_distance:
        return True

    return False


class Frame():
    def __init__(self):
        self.pose = PoseStamped()
        self.absolut_vector = 0

    def __str__(self):
        return "{}\nabsolut_vectro: {}" .format(self.pose, self.absolut_vector)

    def set_pose(self, pose):
        self.pose = pose

        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        z = self.pose.pose.position.z

        self.absolut_vector = math.sqrt(x**2 + y**2 + z**2)

    def get_pose(self):
        return self.pose

    def get_absolut_vector(self):
        return self.absolut_vector



class Measure(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded',
                                       'aborted'],
                             input_keys=['pose_base_cur'],
                             output_keys=['list_detected_frames_tmp'])

        self.max_objects = 5
        self.time_to_measure = 1
        self.max_distance_between_frames = 0.05 # 5cm

    def execute(self, userdata):

        ##detect current objects (related to "camera_rgb_optical_frame")
        rospy.loginfo('MEASURE: record objects in "camera_rgb_optical_frame')
        list_detected_frames = []
        list_detected_frames_tmp = []
        list_detected_frames_tmp = self.record_frame_list(list_detected_frames)

        ##transform current objects (related to "x")
        rospy.loginfo('MEASURE: transform objects related to "map"')
        list_detected_frames = self.transform_frame_list(list_detected_frames_tmp)


        userdata.list_detected_frames_tmp = list_detected_frames_tmp

        rospy.loginfo('MEASURE: SUCCEEDED')
        return 'succeeded'


    def record_frame_list(self, frame_list):

        recognized_object_array = RecognizedObjectArray()

        start_time = rospy.get_time()
        while (rospy.get_time() - start_time) < self.time_to_measure and len(frame_list) < self.max_objects:
            try:
                recognized_object_array = rospy.wait_for_message('/recognized_object_array',
                                                                 RecognizedObjectArray, timeout=2)
                recognized_object =  str(recognized_object_array.objects)

            except rospy.ROSException as e:
                if 'timeout exceeded' in e.message:
                    rospy.loginfo('no object detected! ')
                    continue  # no object detected, continue with trying
                else:
                    raise e

            index = 0
            index_pos = 1
            while index_pos > 0:

                index_pos = recognized_object.find("position", index)
                index_ori = recognized_object.find("orientation", index)
                index_cov = recognized_object.find("covariance", index)
                index = index + index_pos

                # rospy.loginfo('index: \n{}' .format(index))
                # rospy.loginfo('index_pos: \n{}' .format(index_pos))

                object_pose = PoseStamped()
                object_pose.header.frame_id = "camera_rgb_optical_frame"

                if index_pos > 0:
                    obj_pos = str(recognized_object[index_pos:index_ori])
                    obj_ori = str(recognized_object[index_ori:index_cov])

                    match = re.findall(r'-?\d+\.?\d*', str(obj_pos))
                    object_pose.pose.position.x = float(match[0])
                    object_pose.pose.position.y = float(match[1])
                    object_pose.pose.position.z = float(match[2])

                    match = re.findall(r'-?\d+\.?\d*', str(obj_ori))
                    object_pose.pose.orientation.x = float(match[0])
                    object_pose.pose.orientation.y = float(match[1])
                    object_pose.pose.orientation.z = float(match[2])
                    object_pose.pose.orientation.w = float(match[3])

                    frame = Frame()
                    frame.set_pose(object_pose)
                    # rospy.loginfo('obj: \n{}' .format(obj))

                    # append only if object not exist
                    for frame_tar in frame_list:
                        if compare_objects(frame_tar, frame, self.max_distance_between_frames):
                            break
                    else:
                        frame_list.append(frame)
                        rospy.loginfo("append object to list")

        return frame_list


    def sort_frames_by_distance(self, frame_list):

        frame_sub_list_tmp = sorted(frame_list, key=getKey)

        return frame_sub_list_tmp


    def transform_frame_list(self, frame_list):

        frame_list_tmp = []

        for frame in frame_list:
            if frame.pose.header.frame_id == "camera_rgb_optical_frame":
                pose = PoseStamped()
                pose = frame.get_pose()
                frame.set_pose(self.transform_pose_related_to_map(pose))
                frame_list_tmp.append(frame)

        return frame_list_tmp


    def transform_pose_related_to_map(self, pose):
        msg = PoseStamped()
        pose_tar = PoseStamped()
        msg = pose

        listener = tf.TransformListener()

        rospy.loginfo('MEASURE: transforming frames to "map"')
        rospy.loginfo('MEASURE: proccess ..')
        done = False
        while not done:

            try:
                pose_tar = listener.transformPose('map', msg)
                done = True

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

        rospy.loginfo('MEASURE: transform done')
        return pose_tar


class Detect(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded',
                                       'aborted'])

    def execute(self, userdata):

        rospy.loginfo("DETECT: Wait for spotting object..")
        recognized_object_array = RecognizedObjectArray()

        while 1:
            try:
                recognized_object_array = rospy.wait_for_message('/recognized_object_array',
                                                                 RecognizedObjectArray, timeout=2)
                recognized_object =  str(recognized_object_array.objects)

            except rospy.ROSException as e:
                if 'timeout exceeded' in e.message:
                    rospy.loginfo('no object detected! ')
                    continue  # no object detected, continue with trying
                else:
                    raise e

            index_pos = 0
            index_pos = recognized_object.find("position", index_pos)
            if index_pos > 0:
                rospy.loginfo("DETECT: Spot an Object")
                break;


        rospy.loginfo("DETECT: SUCCEEDED")
        return 'succeeded'
