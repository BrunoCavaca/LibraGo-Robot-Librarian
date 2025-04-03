#!/usr/bin/env python3

import rospy
import smach
import math
import sys
import moveit_commander
import numpy as np
from common.navigate import MoveTIAGo
from common.controller_manager import ControllerManager
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped , PoseStamped, Twist # for robot pose
from moveit_commander import PlanningSceneInterface
from tf.transformations import quaternion_from_euler

class DropBook(smach.State):

    def __init__(self, mediator):
        smach.State.__init__(self,
                            outcomes=['gripperSuccess', 'gripperFailure'])
        self.torsoControllerManager = ControllerManager('torso')
        self.gripperControllerManager = ControllerManager('gripper')
        self.velocity_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist,queue_size=20)
        self.mediator = mediator

    def execute(self,userdata):
        if self.move_arm_torso():
            return 'gripperSuccess'   
        return 'gripperFailure'

    def move_arm_torso(self):
        # checks if gripper is empty before initating drop-off sequence
        if self.mediator.getGripperStatus():
            return False

        # Initialize MoveIt! 
        self.reach_max()
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group = moveit_commander.MoveGroupCommander("arm_torso")
        # Set the planning frame and end effector
        group.set_pose_reference_frame("base_footprint")
        group.set_end_effector_link("gripper_grasping_frame")
        # group.set_max_velocity_scaling_factor(0.5)

        # Wait for the planning scene to update
        rospy.sleep(2)

        # Create the goal pose for the end-effector
        #1. Sample goal further away and then move forward slowly w/ gripper in place
        currentPose = group.get_current_pose().pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "base_footprint"
        goal_pose.pose.position.x = currentPose.position.x + 0.2
        goal_pose.pose.position.y = currentPose.position.y
        goal_pose.pose.position.z = 0.75
        print(f"x {goal_pose.pose.position.x} y:{goal_pose.pose.position.y} z:{goal_pose.pose.position.z}")

        # Convert roll, pitch, yaw to quaternion
        quaternion = quaternion_from_euler(0, 1.5708, 0)
        goal_pose.pose.orientation.x = 0
        goal_pose.pose.orientation.y =  quaternion[1]
        goal_pose.pose.orientation.z =  0
        goal_pose.pose.orientation.w = 0
        
        # Set the goal pose and log the target
        group.set_pose_target(goal_pose)

        # Execute the plan
        group.go(wait=True)
        
        group.stop()

        # Clear the target pose
        group.clear_pose_targets()

        self.openGripper()
        self.move_away()
        # Execute the plan

        return True

    def reach_max(self):
        jointInformation = {}
        jointInformation['jointName'] = ['torso_lift_joint']
        jointInformation['positions'] = [0.35]
        self.torsoControllerManager.execute(jointInformation)

    def openGripper(self):
        jointInformation = {}
        jointInformation['jointName'] = ['gripper_finger_joint']
        jointInformation['positions'] = [0.0]
        return self.gripperControllerManager.execute(jointInformation)

    def move_away(self):
        small_distance = 0.2
        vel_msg = Twist()
        vel_msg.linear.x = -0.1
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0

        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0
        rate = rospy.Rate(5)
        while small_distance > 0.03:
            self.velocity_pub.publish(vel_msg)
            rate.sleep()
            small_distance -= 0.025
            rospy.loginfo("MSG SENT")
        
        vel_msg.linear.x = 0.0
        self.velocity_pub.publish(vel_msg)
        rospy.loginfo("REACHED")
        return True