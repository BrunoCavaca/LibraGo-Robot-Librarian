#!/usr/bin/env python3

import rospy
import smach
import ros_numpy
import os
import numpy as np
import easyocr
import cv2
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from sensor_msgs.msg import PointCloud2
from ultralytics import YOLO
from fuzzywuzzy import fuzz
from scipy.spatial.transform import Rotation as R
import math
import tf2_ros
from tf.transformations import *
from tf2_geometry_msgs import *
from geometry_msgs.msg import PoseWithCovarianceStamped  # for robot pose
from pathlib import Path
BOOK_ID = 0

segmentation_model = YOLO("yolo_custom_books.pt")

"""
Detects the books in the scene and checks if they match the user's request
"""
class DetectBooks(smach.State):
    
    def __init__(self, mediator):
        smach.State.__init__(self, 
                             outcomes=['detectSuccess', 'detectFailure'])
        # Initialize the reader and YOLO model
        self.reader = easyocr.Reader(['en'])
        self.mediator = mediator

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.isRunning = False
        self.image_received = False
        self.xyz = None
        self.rgb = None
        self.robot_pose = None

        rospy.Subscriber("/xtion/depth_registered/points", PointCloud2, self.point_cloud_callback)
        rospy.Subscriber("/robot_pose", PoseWithCovarianceStamped, self.robot_pose_callback)


    def robot_pose_callback(self, msg):
        # Stores the robot's pose at each point of detection
        if not self.isRunning:
            self.robot_pose = msg.pose.pose

    def point_cloud_callback(self, msg):
        # stores pointcloud data
        if not self.isRunning:
            self.xyz, self.rgb = self.pointcloud2_to_array(msg)
            self.image_received = True

    def execute(self, userdata):
        bookName = self.mediator.getRequestedBookName()
        # Wait for the image to be received at least once
        while not self.image_received:
            rospy.sleep(0.1)  # Sleep a little to prevent busy waiting
        
        
        # Continuously run YOLO detection on new images
        while not self.mediator.getNavigationStatus():
            if self.xyz is not None and self.rgb is not None and not self.isRunning:
                self.isRunning = True
                result = segmentation_model(self.rgb)
                classes = result[0].boxes.cls.cpu().numpy().astype(int)
                for index, class_id in enumerate(classes):
                    if class_id == BOOK_ID:
                        mask = result[0].masks.data.cpu().numpy()[index, :, :].astype(int)
                        mask_expanded = np.stack([mask, mask, mask], axis=2)
                        obj_rgb = self.rgb * mask_expanded
                        obj_rgb = obj_rgb.astype(np.uint8)
                        
                        img = cv2.cvtColor(obj_rgb, cv2.COLOR_BGR2GRAY)
                        similarityScore = self.runSimilarityChecker(img,bookName)
                        if similarityScore >= 45:
                            self.mediator.setRobotPose([self.robot_pose.position.x,self.robot_pose.position.y,self.robot_pose.position.z])
                            self.mediator.setDetectionStatus()
                            self.isRunning = False
                            print("Valid Detection ocurred at:")
                            print(self.robot_pose)
                            return 'detectSuccess'
                    self.isRunning = False
                self.isRunning = False
                # Sleep to give the loop a pause before processing the next frame
                rospy.sleep(0.75)
        return 'detectFailure'

    def pointcloud2_to_array(self, pointcloud2: PointCloud2) -> tuple:
        """
        Convert a ROS PointCloud2 message to a numpy array.
        """
        pc_array = ros_numpy.point_cloud2.pointcloud2_to_array(pointcloud2)
        split = ros_numpy.point_cloud2.split_rgb_field(pc_array)
        rgb = np.stack([split["b"], split["g"], split["r"]], axis=2)
        xyz = ros_numpy.point_cloud2.get_xyz_points(pc_array, remove_nans=False)
        xyz = np.array(xyz).reshape((pointcloud2.height, pointcloud2.width, 3))
        nan_rows = np.isnan(xyz).all(axis=2)
        xyz[nan_rows] = [0, 0, 0]
        rgb[nan_rows] = [0, 0, 0]
        return xyz, rgb

    def runSimilarityChecker(self,img, bookName):
        text = self.reader.readtext(img, detail=0)
        wordsMatched = ' '.join(text).lower()
        print(f"Matched: {wordsMatched}")
        similarityScore = fuzz.token_sort_ratio(bookName, wordsMatched.lower())
        return similarityScore

