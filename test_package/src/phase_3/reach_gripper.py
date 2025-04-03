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

class ReachGripper(smach.State):

    def __init__(self, mediator):
        smach.State.__init__(self,
                            outcomes=['gripperSuccess', 'gripperFailure'])
        self.mediator = mediator
        self.torsoControllerManager = ControllerManager('torso')
        self.gripperControllerManager = ControllerManager('gripper')
        self.velocity_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist,queue_size=20)
        self.distanceFromBook = 0
        self.distanceCopy = 0

    def execute(self,userdata):
        if self.move_arm_torso():
            self.mediator.setNavigationStatus()
            return 'gripperSuccess'   
        return 'gripperFailure'

    def move_arm_torso(self):
        # Initialize MoveIt! 
        pose = self.mediator.getBookPosition()

        rospy.loginfo(pose)


        # Create the goal pose for the end-effector
        #1. Sample goal further away and then move forward slowly w/ gripper in place
        self.reach_max()
        rospy.sleep(0.75)

        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group = moveit_commander.MoveGroupCommander("arm_torso")
        # Set the planning frame and end effector
        group.set_pose_reference_frame("base_footprint")
        group.set_end_effector_link("gripper_grasping_frame")
        group.set_max_velocity_scaling_factor(1.0)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "base_footprint"
        goal_pose.pose.position.x = round(pose[0] - 0.45,2)
        goal_pose.pose.position.y = round(pose[1] - 0.25,2)
        goal_pose.pose.position.z = round(pose[2] + 0.12,2)

        self.distanceFromBook = (pose[0] - round(pose[0] - 0.45,2))
        self.distanceCopy = self.distanceFromBook
        print(f"x {goal_pose.pose.position.x} y:{goal_pose.pose.position.y} z:{goal_pose.pose.position.z}")

        # Convert roll, pitch, yaw to quaternion
        quaternion = quaternion_from_euler(1.57, 0,0)
        print(f"QUATER: {quaternion}")
        goal_pose.pose.orientation.x = round(quaternion[0],2)
        goal_pose.pose.orientation.y =  0
        goal_pose.pose.orientation.z =  0
        goal_pose.pose.orientation.w = round(quaternion[3],2)
        rospy.sleep(0.5)
        # Set the goal pose and log the target
        group.set_pose_target(goal_pose)

        # Execute the plan
        group.go(wait=True)
        
        group.stop()

        self.approach_book()
        rospy.sleep(1)
        self.closeGripper()
        rospy.sleep(1)
        #move arm up a little with book in gripper
        self.move_away()
        rospy.sleep(1)
        self.move_arm_after_grasp()    
        self.reach_max()
        return True

    def move_arm_after_grasp(self):
        # Small adjustment to the current pose to move the arm
        # This assumes the robot's arm is controlled via MoveIt! and its planning group is set to "arm_torso"

        # Get the current pose of the arm
        group = moveit_commander.MoveGroupCommander("arm_torso")
        current_pose = group.get_current_pose().pose

        # Log current pose for debugging
        rospy.loginfo("Current Arm Pose: %s", current_pose)

        # Define a small offset to move the arm after the grasp
        new_pose = PoseStamped()
        new_pose.header.frame_id = "base_footprint"
        new_pose.pose.position.x = current_pose.position.x - 0.1
        new_pose.pose.position.y = 0
        new_pose.pose.position.z = 0.9 # Move upwards by 10 cm (adjust this as needed)
        
        quaternion = quaternion_from_euler(1.5708, 0, 0)
        new_pose.pose.orientation.x = current_pose.orientation.x
        new_pose.pose.orientation.y =  current_pose.orientation.y
        new_pose.pose.orientation.z =  current_pose.orientation.z
        new_pose.pose.orientation.w = current_pose.orientation.w
        # Log the new pose for debugging
        rospy.loginfo("New Arm Pose: %s", new_pose.pose)

        # Set the new pose target and execute the movement
        group.set_pose_target(new_pose)
        group.go(wait=True)
        
        # Stop the movement after it completes
        group.stop()

        rospy.loginfo("Arm moved slightly after grasp.")
        return True

    def reach_max(self):
        jointInformation = {}
        jointInformation['jointName'] = ['torso_lift_joint']
        jointInformation['positions'] = [0.35]
        self.torsoControllerManager.execute(jointInformation)

    def approach_book(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.1
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0

        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0

        rate = rospy.Rate(5)
        while self.distanceFromBook > 0.03:
            self.velocity_pub.publish(vel_msg)
            rate.sleep()
            self.distanceFromBook -= 0.0225
            rospy.loginfo("MSG SENT")
        
        vel_msg.linear.x = 0.0
        self.velocity_pub.publish(vel_msg)
        rospy.loginfo("REACHED")
        return True
    
    def closeGripper(self):
        jointInformation = {}
        jointInformation['jointName'] = ['gripper_finger_joint']
        jointInformation['positions'] = [0.7]
        return self.gripperControllerManager.execute(jointInformation)
    
    def move_away(self):
        vel_msg = Twist()
        vel_msg.linear.x = -0.1
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0

        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0
        rate = rospy.Rate(5)
        while self.distanceCopy > 0.03:
            self.velocity_pub.publish(vel_msg)
            rate.sleep()
            self.distanceCopy -= 0.025
            rospy.loginfo("MSG SENT")
        
        vel_msg.linear.x = 0.0
        self.velocity_pub.publish(vel_msg)
        rospy.loginfo("REACHED")
        return True