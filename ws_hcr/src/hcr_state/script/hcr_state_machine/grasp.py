#!/usr/bin/env python

import rospy
import smach
import smach_ros

import tf
import math

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

__all__ = ['Pick','Execute','Place']

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a
  tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class Pick(smach.State):
    def __init__(self):
        super(Pick, self).__init__()
        smach.State.__init__(self,
                             outcomes=['succeeded',
                                       'aborted'],
                             input_keys=['pose_object_tar'])

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "hcr_arm"
        self.move_group_arm = moveit_commander.MoveGroupCommander(self.group_name)

        self.group_name = "hcr_gripper"
        self.move_group_gripper = moveit_commander.MoveGroupCommander(self.group_name)

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)




        self.time_out =5.0 #sec

        """
        general
        """
        rospy.loginfo('============ Move Groupe Settings \n')
        rospy.loginfo('\n ============ general Parameter:')

        self.move_group_arm.allow_looking(True)
        self.move_group_arm.allow_replanning(True)

        # rospy.loginfo('============ allow_looking: true')
        # rospy.loginfo('============ allow_replanning: true')

        # self.move_group_arm.set_workspace(ws)

        # self.move_group_arm.set_max_acceleration_scaling_factor(value)
        # self.move_group_arm.set_max_velocity_scaling_factor(value)

        """
        planner
        """
        rospy.loginfo('\n============ planner Parameter:')

        self.move_group_arm.set_num_planning_attempts(6)
        # self.move_group_arm.set_planner_id(planner_id)
        self.move_group_arm.set_planning_time(6) #sec
        # self.move_group_arm.set_pose_reference_frame(/base_footprint)
        # self.move_group_arm.set_end_effector_link(/link_arm6)

        rospy.loginfo('============ set_num_planning_attempts: 3')
        # rospy.loginfo('============ get_planner_id: {}'
        #               .format(self.move_group_arm.get_planner_id()))
        rospy.loginfo('============ get_planning_time: {}'
                      .format(self.move_group_arm.get_planning_time()))
        rospy.loginfo('============ get_planning_frame: {}'
                      .format(self.move_group_arm.get_planning_frame()))
        rospy.loginfo('============ get_pose_reference_frame: {}'
                      .format(self.move_group_arm.get_pose_reference_frame()))
        rospy.loginfo('============ get_end_effector_link: {}'
                      .format(self.move_group_arm.get_end_effector_link()))

        """
        constrains
        """
        rospy.loginfo('\n============ constrains Parameter:')

        self.move_group_arm.clear_path_constraints()
        # self.move_group_arm.clear_trajectory_constraints()

        # self.move_group_arm.set_path_constraints(value)
        # self.move_group_arm.set_trajectory_constraints(value)


        rospy.loginfo('============ get_path_constraints: {}'
                      .format(self.move_group_arm.get_path_constraints()))
        # rospy.loginfo('============ get_trajectory_constraints: {}'
        #               .format(self.move_group_arm.get_trajectory_constraints()))


        """
        target
        """
        rospy.loginfo('\n============ targets Parameter:')

        # self.move_group_arm.set_goal_tolerance(value)
        # self.move_group_arm.set_goal_joint_tolerance(value)
        # self.move_group_arm.set_goal_position_tolerance(value)
        # self.move_group_arm.set_goal_orientation_tolerance(value)

        rospy.loginfo('============ get_goal_tolerance: {}'
                      .format(self.move_group_arm.get_goal_tolerance()))
        rospy.loginfo('============ get_goal_joint_tolerance: {}'
                      .format(self.move_group_arm.get_goal_joint_tolerance()))
        rospy.loginfo('============ get_goal_position_tolerance: {}'
                      .format(self.move_group_arm.get_goal_position_tolerance()))
        rospy.loginfo('============ get_goal_orientation_tolerance: {}'
                      .format(self.move_group_arm.get_goal_orientation_tolerance()))
        rospy.loginfo('============ "arm" get_remembered_joint_values: {}'
                      .format(self.move_group_arm.get_remembered_joint_values()))
        rospy.loginfo('============ "gripper" get_remembered_joint_values: {}'
                      .format(self.move_group_gripper.get_remembered_joint_values()))




        self.pose_arm_home = []
        self.pose_arm_home.append(1.5707)
        self.pose_arm_home.append(-1.5707)
        self.pose_arm_home.append(-1.5707)
        self.pose_arm_home.append(-0.7853)
        self.pose_arm_home.append(1.5707)

        self.pose_arm_intermediate = []
        self.pose_arm_intermediate.append(1.0011)
        self.pose_arm_intermediate.append(0.4538)
        self.pose_arm_intermediate.append(-1.7458)
        self.pose_arm_intermediate.append(-1.0742)
        self.pose_arm_intermediate.append(1.5707)

        self.pose_arm_intermediate2 = []
        self.pose_arm_intermediate2.append(0.3797)
        self.pose_arm_intermediate2.append(1.1476)
        self.pose_arm_intermediate2.append(-1.3615)
        self.pose_arm_intermediate2.append(-2.0371)
        self.pose_arm_intermediate2.append(1.5707)

        self.pose_arm_prePosition = []
        self.pose_arm_prePosition.append(0)
        self.pose_arm_prePosition.append(0.4786)
        self.pose_arm_prePosition.append(-1.6818)
        self.pose_arm_prePosition.append(-1.8308)
        self.pose_arm_prePosition.append(1.5707)




        # move_group "hcr_gripper" open (max 72 deg)
        self.pose_gripper_open = []
        self.pose_gripper_open.append(1.2566370614359172)

        # move_group "hcr_gripper" close (22 deg)
        self.pose_gripper_close = []
        self.pose_gripper_close.append(0.3839724354387525)


    def move_arm_to_joint_goal(self, joint_goal):

        self.move_group_arm.set_joint_value_target(joint_goal)
        rospy.loginfo('============ "hcr_arm" process..')

        while not self.move_group_arm.go(wait=True):
            pass
        else:
            rospy.loginfo('============ done')
            self.move_group_arm.stop()

        current_joints = self.move_group_arm.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.05)


    def move_gripper_to_joint_goal(self, joint_goal):

        self.move_group_gripper.set_joint_value_target(joint_goal)
        rospy.loginfo('============ "hcr_arm" process..')
        self.move_group_gripper.go(wait=True)
        rospy.loginfo('============ done')
        self.move_group_arm.stop()

        # for testing:
        current_joints = self.move_group_gripper.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.05)

    def move_to_target_pose(self, pose):


        frame_plan = self.move_group_arm.get_planning_frame()
        rospy.loginfo('self.move_group_arm.get_planning_frame:\n{}\n'
                      .format(frame_plan))

        frame_ref = self.move_group_arm.get_pose_reference_frame()
        rospy.loginfo('self.move_group_arm.get_pose_reference_frame:\n{}\n'
                      .format(frame_ref))

        frame_eef = self.move_group_arm.get_end_effector_link()
        rospy.loginfo('self.move_group_arm.get_end_effector_link:\n{}\n'
                      .format(frame_eef))


        pose_target = geometry_msgs.msg.PoseStamped()
        pose_target  = self.move_group_arm.get_current_pose()
        pose_target.pose.position.x = pose.position.x
        pose_target.pose.position.y = pose.position.y
        rospy.loginfo('pose target: link_arm6 after rpy:\n {}\n' .format(pose_target))

        rospy.loginfo('============ "hcr_arm" process..')
        self.move_group_arm.set_joint_value_target(pose_target, True)

        self.move_group_arm.set_start_state_to_current_state()

        start_time = rospy.get_time()
        while (rospy.get_time() - start_time) < self.time_out and not self.move_group_arm.go(wait=True):
            pass
        else:
            self.move_group_arm.stop()

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal = pose_target.pose

        current_pose = self.move_group_arm.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.02)


    def execute(self, userdata):

        rospy.loginfo('============ move home')
        succeed = self.move_arm_to_joint_goal(self.pose_arm_home)
        rospy.loginfo('done: {}\n' .format(succeed))

        rospy.loginfo('============ move intermediate')
        succeed = self.move_arm_to_joint_goal(self.pose_arm_intermediate)
        rospy.loginfo('done: {}\n' .format(succeed))

        rospy.loginfo('============ move intermediate2')
        succeed = self.move_arm_to_joint_goal(self.pose_arm_intermediate2)
        rospy.loginfo('done: {}\n' .format(succeed))

        rospy.loginfo('============ open gripper')
        succeed = self.move_gripper_to_joint_goal(self.pose_gripper_open)
        rospy.loginfo('done: {}\n' .format(succeed))

        rospy.loginfo('============ move pre position')
        succeed = self.move_arm_to_joint_goal(self.pose_arm_prePosition)
        rospy.loginfo('done: {}\n' .format(succeed))

        rospy.loginfo('============ move target pose')
        pose_goal = copy.deepcopy(userdata.pose_object_tar.pose)
        rospy.loginfo('target object pose:\n{}\n' .format(userdata.pose_object_tar))
        succeed = self.move_to_target_pose(pose_goal)
        rospy.loginfo('done: {}\n' .format(succeed))

        rospy.loginfo('============ close gripper')
        test = self.move_gripper_to_joint_goal(self.pose_gripper_close)
        rospy.loginfo('done: {}\n' .format(test))

        rospy.loginfo('============ move home')
        test = self.move_arm_to_joint_goal(self.pose_arm_home)
        rospy.loginfo('done: {}\n' .format(test))


        rospy.loginfo('PICK: SUCCEEDED')
        return 'succeeded'




class Place(smach.State):
    def __init__(self):
        super(Place, self).__init__()
        smach.State.__init__(self,
                             outcomes=['succeeded','aborted'])
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "hcr_arm"
        self.move_group_arm = moveit_commander.MoveGroupCommander(self.group_name)

        self.group_name = "hcr_gripper"
        self.move_group_gripper = moveit_commander.MoveGroupCommander(self.group_name)

        self.pose_arm_home = []
        self.pose_arm_home.append(1.5707)
        self.pose_arm_home.append(-1.5707)
        self.pose_arm_home.append(-1.5707)
        self.pose_arm_home.append(-0.7853)
        self.pose_arm_home.append(1.5707)

        self.pose_arm_place = []
        self.pose_arm_place.append(0)
        self.pose_arm_place.append(-0.9834)
        self.pose_arm_place.append(-0.817)
        self.pose_arm_place.append(-1.0055)
        self.pose_arm_place.append(1.5707)

        self.pose_arm_place2 = []
        self.pose_arm_place2.append(0.2762)
        self.pose_arm_place2.append(0.1565)
        self.pose_arm_place2.append(-1.3295)
        self.pose_arm_place2.append(-1.9225)
        self.pose_arm_place2.append(1.5707)

        self.pose_gripper_open = []
        self.pose_gripper_open.append(1.2566370614359172)

        self.pose_gripper_close = []
        self.pose_gripper_close.append(0.3839724354387525)


    def move_arm_to_joint_goal(self, joint_goal):

        self.move_group_arm.set_joint_value_target(joint_goal)
        rospy.loginfo('============ "hcr_arm" process..')
        while not self.move_group_arm.go(wait=True):
            pass
        else:
            rospy.loginfo('============ done')
            self.move_group_arm.stop()

        current_joints = self.move_group_arm.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.05)


    def move_gripper_to_joint_goal(self, joint_goal):

        self.move_group_gripper.set_joint_value_target(joint_goal)
        rospy.loginfo('============ "hcr_arm" process..')
        self.move_group_gripper.go(wait=True)
        rospy.loginfo('============ done')
        self.move_group_arm.stop()

        # for testing:
        current_joints = self.move_group_gripper.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.05)


    def execute(self, userdata):

        rospy.loginfo('============ move home')
        succeed = self.move_arm_to_joint_goal(self.pose_arm_home)
        rospy.loginfo('done: {}\n' .format(succeed))

        rospy.loginfo('============ move place')
        succeed = self.move_arm_to_joint_goal(self.pose_arm_place)
        rospy.loginfo('done: {}\n' .format(succeed))

        rospy.loginfo('============ open gripper')
        succeed = self.move_gripper_to_joint_goal(self.pose_gripper_open)
        rospy.loginfo('done: {}\n' .format(succeed))

        rospy.loginfo('============ move place2')
        succeed = self.move_arm_to_joint_goal(self.pose_arm_place2)
        rospy.loginfo('done: {}\n' .format(succeed))

        rospy.loginfo('============ move home')
        succeed = self.move_arm_to_joint_goal(self.pose_arm_home)
        rospy.loginfo('done: {}\n' .format(succeed))

        rospy.loginfo('============ close gripper')
        test = self.move_gripper_to_joint_goal(self.pose_gripper_close)
        rospy.loginfo('done: {}\n' .format(test))

        rospy.loginfo('PLACE: SUCCEEDED')
        return 'succeeded'



