#!/usr/bin/env python3
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint



class ControllerManager():
    def __init__(self,hardwareUnit):
        self._client = actionlib.SimpleActionClient(f"{hardwareUnit}_controller/follow_joint_trajectory",
                                                    FollowJointTrajectoryAction)
        self._client.wait_for_server()

    def execute(self, jointInformation):
        # Prepare goal for torso controller
        goal = FollowJointTrajectoryGoal()
        print(jointInformation['jointName'])
        goal.trajectory.joint_names = jointInformation['jointName']

        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = jointInformation['positions']
        point.time_from_start = rospy.Duration(1)  # Time to reach the goal position (1 second)

        goal.trajectory.points.append(point)

        # Send the goal to the action server and wait for the result
        self._client.send_goal(goal)
        self._client.wait_for_result()

        # Check if the action was successful
        return self._client.get_state() == actionlib.GoalStatus.SUCCEEDED

    def resetHead(self):
        goal = FollowJointTrajectoryGoal()

        goal.trajectory.joint_names = ['head_1_joint','head_2_joint']

        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = [0.00,0.00]
        point.time_from_start = rospy.Duration(1)  # Time to reach the goal position (1 second)

        goal.trajectory.points.append(point)

        # Send the goal to the action server and wait for the result
        self._client.send_goal(goal)
        self._client.wait_for_result()

        # Check if the action was successful
        return self._client.get_state() == actionlib.GoalStatus.SUCCEEDED