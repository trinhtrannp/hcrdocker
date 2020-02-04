#!/usr/bin/env python

import rospy
import smach     # StateMachine, Concurrence
import smach_ros # ServiceState, SimpleActionState, MonitorState, IntrospectionServer

import threading
# import tf
import tf_conversions
import tf2_ros

import geometry_msgs.msg

# Statemachine classes
import logic
import move
import measure
import grasp

def main():
    rospy.init_node('hcr_state_node')

    sm_hcr = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    """
    Initialize userdata
    """
    # target and current positions
    pose_init = geometry_msgs.msg.PoseStamped()
    pose_init.pose.position.x = 0
    pose_init.pose.position.y = 0
    pose_init.pose.position.z = 0
    pose_init.pose.orientation.x = 0
    pose_init.pose.orientation.y = 0
    pose_init.pose.orientation.z = 0
    pose_init.pose.orientation.w = 1

    sm_hcr.userdata.pose_base_cur = geometry_msgs.msg.PoseStamped()
    sm_hcr.userdata.pose_base_tar = geometry_msgs.msg.PoseStamped()
    sm_hcr.userdata.pose_object_cur = geometry_msgs.msg.PoseStamped()
    sm_hcr.userdata.pose_object_tar = geometry_msgs.msg.PoseStamped()

    sm_hcr.userdata.pose_base_cur = pose_init
    sm_hcr.userdata.pose_base_cur.header.frame_id = "map"
    sm_hcr.userdata.pose_base_tar = pose_init
    sm_hcr.userdata.pose_base_tar.header.frame_id = "map"

    sm_hcr.userdata.pose_object_cur = pose_init
    sm_hcr.userdata.pose_object_tar = pose_init


    # Flags (flg)
    sm_hcr.userdata.flg_move_detect_done = False
    sm_hcr.userdata.flg_move_search_done = False
    sm_hcr.userdata.flg_object_grasp_done = False
    sm_hcr.userdata.flg_object_place_done = False

    sm_hcr.userdata.flg_move_new_goal = False



    # List of detected frames (type class Frame) for each run
    sm_hcr.userdata.list_detected_frames_tmp = []
    sm_hcr.userdata.list_detected_frames_1 = []
    sm_hcr.userdata.list_detected_frames_2 = []
    sm_hcr.userdata.list_detected_frames_3 = []

    """
    Initialize subscribers
    """
    def _move_base_goal_cb(pose):
        sm_hcr.userdata.pose_base_tar = pose
        sm_hcr.userdata.flg_move_new_goal = True

    rospy.Subscriber('/move_base_simple/goal', geometry_msgs.msg.PoseStamped, _move_base_goal_cb)

    """
    Initialize thread
    """
    def thread_tf_target_object_pose():
        while 1:
            br = tf2_ros.TransformBroadcaster()
            t = geometry_msgs.msg.TransformStamped()

            t.header.stamp = rospy.Time.now()
            t.header.frame_id = sm_hcr.userdata.pose_object_tar.header.frame_id
            t.child_frame_id = "target_object"
            t.transform.translation = sm_hcr.userdata.pose_object_tar.pose.position
            t.transform.rotation = sm_hcr.userdata.pose_object_tar.pose.orientation

            try:
                br.sendTransform(t)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("ERROR: thread_tf_target_object_pose: tf-exeption")

    with sm_hcr:
        ### LOGIC ###
        smach.StateMachine.add('LOGIC', logic.Logic(),
                         transitions={'aborted':'aborted',
                                      'reset':'RESET',
                                      'move_detect':'MOVE_DETECT',
                                      'move_search':'MOVE_SEARCH',
                                      'object_grasp':'OBJECT_GRASP',
                                      'object_place':'OBJECT_PLACE'},
                         remapping={'flg_reset':'flg_reset',
                                    'flg_score':'flg_score',
                                    'flg_move_detect_done':'flg_move_detect_done',
                                    'flg_move_search_done':'flg_move_search_done',
                                    'flg_object_grasp_done':'flg_object_grasp_done',
                                    'flg_object_place_done':'flg_object_place_done'})

        ### RESET ###
        smach.StateMachine.add('RESET', logic.Reset(),
                           transitions={'succeeded':'LOGIC'},
                           remapping={'flg_move_detect_done':'flg_move_detect_done',
                                      'flg_move_search_done':'flg_move_search_done',
                                      'flg_move_new_goal':'flg_move_new_goal',
                                      'flg_object_grasp_done':'flg_object_grasp_done',
                                      'flg_object_place_done':'flg_object_place_done',
                                      'pose_base_tar':'pose_base_tar',
                                      'list_detected_frames_tmp':'list_detected_frames_tmp',
                                      'list_detected_frames_1':'list_detected_frames_1',
                                      'list_detected_frames_2':'list_detected_frames_2',
                                      'list_detected_frames_3':'list_detected_frames_3'})

        ### SCORE ###
        smach.StateMachine.add('SCORE', logic.Score(),
                           transitions={'succeeded':'LOGIC',
                                        'aborted':'RESET'},
                                   remapping={'list_detected_frames_1':'list_detected_frames_1',
                                              'list_detected_frames_2':'list_detected_frames_2',
                                              'list_detected_frames_3':'list_detected_frames_3'})

        ### MOVE_DETECT ###
        sm_MoveDetect = smach.StateMachine(
                outcomes=['succeeded',
                          'aborted'],
                input_keys = ['pose_base_cur','list_detected_frames_tmp'],
                output_keys = ['pose_base_cur','list_detected_frames_tmp',
                               'list_detected_frames_1','flg_move_detect_done'])

        # sub Detect
        smach.StateMachine.add('MOVE_DETECT',sm_MoveDetect,
                         {'succeeded':'SCORE',
                          'aborted':'RESET'})
        with sm_MoveDetect:
            smach.StateMachine.add('DETECT', measure.Detect(),
                                   transitions={'succeeded':'STOP'})

            smach.StateMachine.add('STOP', move.Stop(),
                                   transitions={'succeeded':'MEASURE'},
                                   remapping={'pose_base_cur':'pose_base_cur'})

            smach.StateMachine.add('MEASURE', measure.Measure(),
                                   transitions={'succeeded':'MOVE_DETECT_EXIT'},
                                   remapping={'pose_base_cur':'pose_base_cur',
                                              'list_detected_frames_tmp':'list_detected_frames_tmp'})

            smach.StateMachine.add('MOVE_DETECT_EXIT', logic.MoveDetectExit(),
                                   transitions={'succeeded':'succeeded'},
                                   remapping={'list_detected_frames_tmp':'list_detected_frames_tmp',
                                              'list_detected_frames_1':'list_detected_frames_1',
                                              'flg_move_detect_done':'flg_move_detect_done'})


        ### MOVE_SEARCH ###
        sm_MoveSearch = smach.StateMachine(outcomes=['succeeded',
                                                      'aborted'],
                                            input_keys = ['pose_base_tar',
                                                          'pose_base_cur',
                                                          'pose_object_tar',
                                                          'list_detected_frames_tmp'],
                                            output_keys = ['pose_base_tar',
                                                           'pose_base_cur',
                                                           'list_detected_frames_tmp',
                                                           'list_detected_frames_2',
                                                           'flg_move_search_done'])


        smach.StateMachine.add('MOVE_SEARCH',sm_MoveSearch,
                         {'succeeded':'SCORE',
                          'aborted':'RESET'})

        with sm_MoveSearch:
            smach.StateMachine.add('BASEPOSE_FINDER', logic.BasePoseFinder(),
                                   transitions={'succeeded':'MOVE'},
                                   # transitions={'succeeded':'MEASURE'},
                                   remapping={'pose_object_tar':'pose_object_tar',
                                              'pose_base_tar':'pose_base_tar'})

            smach.StateMachine.add('MOVE', move.Move(),
                                   transitions={'succeeded':'MEASURE'},
                                   remapping={'pose_base_tar':'pose_base_tar',
                                              'pose_base_cur':'pose_base_cur'})

            smach.StateMachine.add('MEASURE', measure.Measure(),
                                   transitions={'succeeded':'MOVE_SEARCH_EXIT'},
                                   remapping={'pose_base_cur':'pose_base_cur',
                                              'list_detected_frames_tmp':'list_detected_frames_tmp'})

            smach.StateMachine.add('MOVE_SEARCH_EXIT', logic.MoveSearchExit(),
                                   transitions={'succeeded':'succeeded'},
                                   remapping={'list_detected_frames_tmp':'list_detected_frames_tmp',
                                              'list_detected_frames_2':'list_detected_frames_2',
                                              'flg_move_search_done':'flg_move_search_done'})


        ### OBJECT_GRASP ###
        sm_ObjectGrasp = smach.StateMachine(outcomes=['succeeded',
                                                      'aborted'],
                                            input_keys=['flg_object_grasp_done',
                                                        'pose_object_tar'],
                                            output_keys=['flg_object_grasp_done'])


        smach.StateMachine.add('OBJECT_GRASP',sm_ObjectGrasp,{'succeeded':'LOGIC',
                                                        'aborted':'RESET'})

        with sm_ObjectGrasp:
            smach.StateMachine.add('PICK', grasp.Pick(),
                                   transitions={'succeeded':'OBJECT_GRASP_EXIT'},
                                   remapping={'pose_object_tar':'pose_object_tar'})

            smach.StateMachine.add('OBJECT_GRASP_EXIT', logic.ObjectGraspExit(),
                                   transitions={'succeeded':'succeeded'},
                                   remapping={'flg_object_grasp_done':'flg_object_grasp_done'})


         ### OBJECT_PLACE ###
        sm_ObjectPlace = smach.StateMachine(outcomes=['succeeded',
                                                      'aborted'],
                                            input_keys=['pose_base_cur',
                                                        'pose_base_tar',
                                                        'flg_object_place_done'],
                                            output_keys=['pose_base_cur',
                                                         'flg_object_place_done'])


        smach.StateMachine.add('OBJECT_PLACE',sm_ObjectPlace,{'succeeded':'LOGIC',
                                                        'aborted':'RESET'})

        with sm_ObjectPlace:
            smach.StateMachine.add('MOVE', move.Move(),
                                   transitions={'succeeded':'PLACE'},
                                   remapping={'pose_base_tar':'pose_base_tar',
                                              'pose_base_cur':'pose_base_cur'})

            smach.StateMachine.add('PLACE', grasp.Place(),
                                   transitions={'succeeded':'OBJECT_PLACE_EXIT'})

            smach.StateMachine.add('OBJECT_PLACE_EXIT', logic.ObjectPlaceExit(),
                                   transitions={'succeeded':'succeeded'},
                                   remapping={'flg_object_place_done':'flg_object_place_done'})



    # # attach a SMACH introspection server
    sis = smach_ros.IntrospectionServer('hcr_state_machine', sm_hcr, '/HCR_State_Machine')
    sis.start()

    # set preempt handler
    smach_ros.set_preempt_handler(sm_hcr)

    # Execute SMACH tree in a separate thread so that we can ctrl-c the script
    thread_tf_tarBasePose = threading.Thread(target=thread_tf_target_object_pose)
    thread_tf_tarBasePose.start()

    thread_smach = threading.Thread(target = sm_hcr.execute)
    thread_smach.start()



    # signal handler
    rospy.spin()


if __name__ == '__main__':
    main()
