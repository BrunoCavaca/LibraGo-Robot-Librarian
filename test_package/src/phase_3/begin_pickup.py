#!/usr/bin/env python3

import rospy
import smach
import math
import numpy as np
from common.navigate import MoveTIAGo
from common.controller_manager import ControllerManager
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped  # for robot pose


SHELVES = [[[-1.9,-2.55,-0.001],[-1.9,-1.59,-0.001],[-1.9,-0.7,0],[-1.9, 0.19, -0.001]],#1st layer shelf coords
            [[0.25, -2.55, -0.001],[0.25, -1.59, -0.001],[0.25, -0.7, 0],[0.25, 0.19, 0]],#2nd layer shelf coords
            [[1.9,-2.55,-0.001],[1.9,-1.59,-0.001],[1.9,-0.7,0],[1.9,0.07,0]]#3rd layer shelf coords           
           ] 

class BeginPickup(smach.State):

    def __init__(self, mediator):
        smach.State.__init__(self,
                            outcomes=['navigateSuccess', 'navigateFailure'])
        self.moveRobot = MoveTIAGo()
        self.controllerManager = ControllerManager('head')
        self.mediator = mediator
        self.robot_pose = None
        self.markerReceived = False
        self.marker_positions = []
        rospy.Subscriber("/robot_pose", PoseWithCovarianceStamped, self.robot_pose_callback)
        self.marker_sub = rospy.Subscriber('/vision/aruco_marker', PointStamped, self.marker_callback)
   
    def robot_pose_callback(self, msg):
        self.robot_pose = msg.pose.pose        

    def marker_callback(self, msg):
        # Save the marker position
        if not self.markerReceived:
            position = (msg.point.x, msg.point.y, msg.point.z)
            self.marker_positions.append(position)
            rospy.loginfo(f"Received Marker Position: {position}")
  
    def execute(self, userdata):
        # Prepare goal for torso controller
        # using {} is more efficient than dict() declaration
        if self.mediator.getDetectionFail():
            newPoint = self.get_idx(self.mediator.getRobotPose())
            if newPoint[1]+1 > 3:            
                nextClosest = SHELVES[newPoint[0]][newPoint[1]-1]
            elif newPoint[1]-1 < 0:
                return 'navigateFailure'
            else:
                nextClosest = SHELVES[newPoint[0]][newPoint[1]+1]
            executed = self.moveRobot.execute_movement(nextClosest,goingRightToLeft=False,facingNorth=True)
            if executed:
                self.mediator.setRobotPose(nextClosest) 
                return 'navigateSuccess'
        else:
            self.controllerManager.resetHead()
            jointInformation = {}
            pose = self.mediator.getRobotPose()
            if pose == self.get_closest_point_from_shelves(pose):
                return 'navigateSuccess'
            vals = self.get_closest_point_from_shelves(pose)
            closest_point = SHELVES[vals[0]][vals[1]]
            executed = self.moveRobot.execute_movement(closest_point,goingRightToLeft=False,facingNorth=True)
            if executed:
                self.mediator.setRobotPose(closest_point)
            return 'navigateSuccess'
        
    def get_idx(self,robot_pose):
        for layer_index, layer in enumerate(SHELVES):
            for coord_index, coord in enumerate(layer):
                if coord == robot_pose:
                    return [layer_index, coord_index]
        return None  # If the coordinate isn't found
    
    def get_closest_point_from_shelves(self, robot_pose):
        """
        Given the robot's pose and the SHELVES array, this function returns the closest point on the shelves.
        
        :param robot_pose: The robot's current pose as a list [x, y, z].
        :return: The closest point from the SHELVES array.
        """
        robot_x, robot_y, robot_z = robot_pose  # Extract the robot's position

        # Initialize variables to keep track of the closest point and minimum distance
        closest_point = []
        min_distance = float('inf')

        # Iterate over each layer and point in SHELVES to find the closest point
        for i in range(len(SHELVES)):
            for j in range(len(SHELVES[0])):
                point_x, point_y, point_z = SHELVES[i][j]

                # Calculate the Euclidean distance between the robot and the current point
                distance = math.sqrt((point_x - robot_x)**2 + (point_y - robot_y)**2)
                print(distance)

                # Update closest point if this one is closer
                if distance < min_distance:
                    min_distance = distance
                    closest_point = [i,j]
            
        if closest_point[0] == 0 and closest_point[1] not in [0,3]:
            closest_point[1] = closest_point[1] - 1
        elif closest_point[0] == 1 and closest_point[1] not in [0,3]:
            closest_point[1] = closest_point[1] + 1
        elif closest_point[0] == 2 and closest_point[1] not in [0,3]:
            closest_point[1] = closest_point[1] - 1
        print(f"CLosest point idx: {closest_point}")
        return closest_point

    # def get_closest_point(self, robot_pose):
    #     """
    #     Given the robot's pose and a list of points, this function returns the closest point.
        
    #     :param robot_pose: The robot's current pose as a list [x, y, z].
    #     :return: The closest point from the list.
    #     """
    #     # Extract robot's position (assuming robot_pose is a list [x, y, z])
    #     robot_x, robot_y, robot_z = robot_pose  # x, y, z coordinates of the robot

    #     # Initialize variables to keep track of the closest point and minimum distance
    #     closest_point = None
    #     min_distance = float('inf')

    #     for point in NEAREST_POINTS:
    #         # Extract point coordinates
    #         point_x, point_y, point_z = point

    #         # Calculate the Euclidean distance between robot and the current point
    #         distance = math.sqrt((point_x - robot_x)**2 + (point_y - robot_y)**2)

    #         # Update closest point if this one is closer
    #         if distance < min_distance:
    #             min_distance = distance
    #             closest_point = point

    #     return closest_point

    # def is_within_bounds(self):
    #     def checkPos(polygon):
    #         """
    #         Ray-casting algorithm to check if a point is inside a polygon.
    #         :param point: (x, y) tuple of the point to check
    #         :param polygon: List of (x, y) tuples defining the polygon
    #         :return: True if point is inside the polygon, False otherwise
    #         """
    #         x, y = self.robot_pose.position.x, self.robot_pose.position.y
    #         inside = False
    #         n = len(polygon)
    #         p1x, p1y = polygon[0]
    #         for i in range(n + 1):
    #             p2x, p2y = polygon[i % n]
    #             if y > min(p1y, p2y):
    #                 if y <= max(p1y, p2y):
    #                     if x <= max(p1x, p2x):
    #                         if p1y != p2y:
    #                             xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
    #                         if p1x == p2x or x <= xinters:
    #                             inside = not inside
    #             p1x, p1y = p2x, p2y
    #         return inside
    #     # Extract the x, y positions of the bounds (ignoring z)
    #     polygon = [(point[0], point[1]) for point in BOUNDS]
        
    #     # Use the point-in-polygon function
    #     return checkPos(polygon)