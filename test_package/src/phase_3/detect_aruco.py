#!/usr/bin/env python3

import rospy
import smach
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
import message_filters
import tf2_ros
import tf2_geometry_msgs
import tf.transformations
import numpy as np
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32

"""
THIS IS FULLY FUNCTIONAL BUT NOT NEEDED ANYMORE
ARUCO MARKER IMPLEMENTATION CODE
"""

# Load ArUco dictionary and parameters
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
MID_SHELF_ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
LOWER_SHELF_ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()

class DetectAruco(smach.State):
    def __init__(self,mediator):
        smach.State.__init__(self,
                    outcomes=['detectSuccess', 'detectFailure','greaterThanOneDetection'])
        # ADD INIT HERE
        # Set up subscribers
        self.rgb_sub = message_filters.Subscriber('/xtion/rgb/image_raw', Image)
        self.depth_sub = message_filters.Subscriber('/xtion/depth_registered/image_raw', Image)
        self.camera_info_sub = rospy.Subscriber('/xtion/rgb/camera_info', CameraInfo, self.camera_info_callback)
        self.topShelfDetector = cv2.aruco.ArucoDetector(ARUCO_DICT,ARUCO_PARAMS)
        self.midShelfDetector = cv2.aruco.ArucoDetector(MID_SHELF_ARUCO_DICT,ARUCO_PARAMS)
        self.mediator = mediator
        # Approximate time synchronizer for RGB and Depth
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.image_callback)

        # Transformation buffer and listener for TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Store the CameraInfo data
        self.camera_info_received = False
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rgb_image_msg = None
        self.depth_image_msg = None
        self.found = False


    def camera_info_callback(self, camera_info_msg):
        if not self.camera_info_received and not self.found:
            self.camera_matrix = np.array(camera_info_msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(camera_info_msg.D)
            self.camera_info_received = True

    def image_callback(self, rgb_img, depth_img):
        if self.camera_info_received and not self.found:
            self.rgb_image_msg = rgb_img
            self.depth_image_msg = depth_img

    def execute(self, userdata):
        # Convert the RGB image to an OpenCV image
        while not self.found:
            self.cv_rgb_image = self.bridge.imgmsg_to_cv2(self.rgb_image_msg, desired_encoding='bgr8')
            cv_depth_image = self.bridge.imgmsg_to_cv2(self.depth_image_msg, desired_encoding='passthrough')
            print("reach here")
            # Detect ArUco markers
            corners, ids, _ = self.topShelfDetector.detectMarkers(self.cv_rgb_image)
            print(f"ids {ids}")
            # If markers are detected
            if ids is not None:
                rvecs,tvecs, _ = self.my_estimatePoseSingleMarkers(corners,0.06, self.camera_matrix, self.dist_coeffs)
                shelfBookIsOn = self.mediator.getShelfCount()
                if len(ids) > 1:
                    return 'greatergreaterThanOneDetection'
                for i in range(len(ids)):
                    if i == (shelfBookIsOn - 1):
                        cv2.aruco.drawDetectedMarkers(self.cv_rgb_image, corners, ids)
                        # Get the translation vector (tvecs) in the camera frame (camera_optical_frame)
                        marker_position_camera = tvecs[i].flatten()

                        # Transform the position directly to base_footprint
                        transformed_position_base, euler_angles = self.transform_to_base_footprint(marker_position_camera, self.rgb_image_msg.header.stamp)

                        if transformed_position_base:
                            # Log and publish the transformed position and marker ID
                            rospy.loginfo(f"Transformed position in base_footprint: {transformed_position_base}")
                            rospy.loginfo(f"Yaw: {euler_angles[0]}, Pitch: {euler_angles[1]}, Roll: {euler_angles[2]}")
                            # Publish the transformed position as PointStamped
                            point_msg = PointStamped()
                            point_msg.header.stamp = self.rgb_image_msg.header.stamp
                            point_msg.header.frame_id = 'base_footprint'
                            point_msg.point.x = transformed_position_base[0]
                            point_msg.point.y = transformed_position_base[1]
                            point_msg.point.z = transformed_position_base[2]

                            self.mediator.setArucoPose(point_msg)
                            self.mediator.setArucoEulerAngles([euler_angles[2],euler_angles[1],euler_angles[0]])
                            self.found = True
                            return 'detectSuccess'
            
                    
    def my_estimatePoseSingleMarkers(self,corners, marker_size, mtx, distortion):
      '''
      This will estimate the rvec and tvec for each of the marker corners detected by:
        corners, ids, rejectedImgPoints = detector.detectMarkers(image)
      corners - is an array of detected corners for each detected marker in the image
      marker_size - is the size of the detected markers
      mtx - is the camera matrix
      distortion - is the camera distortion matrix
      RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
      '''
      marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, -marker_size / 2, 0],
                                [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
      trash = []
      rvecs = []
      tvecs = []
      print("here?")
      for c in corners:
          nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
          rvecs.append(R)
          tvecs.append(t)
          trash.append(nada)
      return rvecs, tvecs, trash

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

            # Extract the rotation as a quaternion
            quaternion = (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            )

            # Convert quaternion to Euler angles (roll, pitch, yaw)
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)

            return transformed_position, (yaw, pitch, roll)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Transform failed: {e}")
            return None, None