#!/usr/bin/env python3

import rospy
import smach
import ros_numpy
import numpy as np
import easyocr
import cv2
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, PointStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2
from ultralytics import YOLO
from fuzzywuzzy import fuzz
from scipy.spatial.transform import Rotation as R
import tf2_ros
import tf2_geometry_msgs
from common.controller_manager import ControllerManager

BOOK_ID = 0
segmentation_model = YOLO("yolo_custom_books.pt")

class DetectBookReRun(smach.State):
    
    def __init__(self, mediator):
        smach.State.__init__(self, 
                             outcomes=['detectSuccess', 'detectFailure'])
        # Initialize the reader and YOLO model
        self.controllerManager = ControllerManager('head')
        self.reader = easyocr.Reader(['en'])
        self.mediator = mediator

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber("/xtion/depth_registered/points", PointCloud2, self.point_cloud_callback)
        rospy.Subscriber("/robot_pose", PoseWithCovarianceStamped, self.robot_pose_callback)

        self.image_received = False
        self.xyz = None
        self.rgb = None
        self.isRunning = False

        self.hdrstamp  = None
        self.booksFound = {}
        self.jointInformation = {}
        # Initialize the subscriber to get the PointCloud2 message
        
        # Subscribe to /robot_pose to get the robot's pose (position and orientation)
        self.robot_pose = None

    def robot_pose_callback(self, msg):
        if not self.isRunning:
            self.robot_pose = msg.pose.pose

    def point_cloud_callback(self, msg):
        if not self.isRunning:
            self.hdrstamp = msg.header.stamp
            self.xyz, self.rgb = self.pointcloud2_to_array(msg)
            self.image_received = True

    # Sets head facing in a downwards position
    def setHeadOrientation(self):
        self.jointInformation['jointName'] = ['head_1_joint','head_2_joint']
        self.jointInformation['positions'] = [0.00,-0.44]
        self.controllerManager.execute(self.jointInformation)
        self.jointInformation.clear()

    def execute(self, userdata):
        self.setHeadOrientation()
        while not self.image_received:
            rospy.sleep(0.1)  # Sleep a little to prevent busy waiting

        runOnce = False
        # Wait for the image to be received at least once
        while not runOnce:
            result = segmentation_model(self.rgb)
            classes = result[0].boxes.cls.cpu().numpy().astype(int)
            for index, class_id in enumerate(classes):
                if class_id == BOOK_ID:
                    mask = result[0].masks.data.cpu().numpy()[index, :, :].astype(int)
                    mask_expanded = np.stack([mask, mask, mask], axis=2)
                    obj_rgb = self.rgb * mask_expanded
                    obj_rgb = obj_rgb.astype(np.uint8)
                    obj_xyz = self.xyz * mask_expanded
                    centroid = self.calculate_centroid(obj_xyz)
                    transformed_centroid = self.transform_to_base_footprint(centroid, self.hdrstamp)

                    img = cv2.cvtColor(obj_rgb, cv2.COLOR_BGR2GRAY)

                    # Run OCR on processed image
                    text = self.reader.readtext(img, detail=0)
                    wordsMatched = ' '.join(text).lower()
                    self.booksFound[wordsMatched] = transformed_centroid
            if self.findClosestMatch():
                return 'detectSuccess'

        self.mediator.getDetectionFail()
        return 'detectFailure'
    

    def findClosestMatch(self):
        """ 
        Find the closest matching book title and stores related info in mediator
        Returns: True if a valid book title was found, False if otherwise
        """

        highestVal = 0
        words = ""
        wordsMatched = sorted(list(self.booksFound.keys()))
        for i in range(len(wordsMatched)):
            similarityScore = fuzz.token_sort_ratio("Science of the bottom line".lower(), wordsMatched[i])
            if similarityScore >= highestVal:
                highestVal = similarityScore
                words = wordsMatched[i]
        
        self.mediator.setBookPosition(self.booksFound[words])
        return True if highestVal >= 0 else False

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

    def calculate_centroid(self, obg_xyz):
        """
        Calculate the centroid of the object by averaging the 3D coordinates.
        """
        # Remove all zero points (where the mask was 0)
        non_zero_points = obg_xyz[np.all(obg_xyz != 0, axis=-1)]
        
        # Calculate the mean of the coordinates
        if non_zero_points.size > 0:
            centroid = np.mean(non_zero_points, axis=0)
            return centroid
        else:
            return None

    def transform_to_base_footprint(self, marker_position, stamp):
            try:
                # Create a PointStamped message for the marker's position in camera_optical_frame
                point_camera_optical = PointStamped()
                point_camera_optical.header.frame_id = 'xtion_optical_frame'
                point_camera_optical.header.stamp = stamp
                point_camera_optical.point.x = marker_position[0]
                point_camera_optical.point.y = marker_position[1]
                point_camera_optical.point.z = marker_position[2]

                # Get the full transform from camera_optical_frame to base_footprint
                transform = self.tf_buffer.lookup_transform('base_footprint', 'xtion_optical_frame', stamp, rospy.Duration(2.0))

                # Transform the point to base_footprint
                transformed_point = tf2_geometry_msgs.do_transform_point(point_camera_optical, transform)

                # Extract the translation part
                transformed_position = [transformed_point.point.x, transformed_point.point.y, transformed_point.point.z]

                return transformed_position

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr(f"Transform failed: {e}")
                return None

