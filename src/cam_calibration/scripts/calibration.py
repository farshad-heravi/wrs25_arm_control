#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf_transformations

class CameraPoseEstimation(Node):
    def __init__(self):
        super().__init__('camera_pose_estimation_node')

        self.bridge = CvBridge()
        self.camera_info = None
        self.calibrated = False
        self.rvec = None
        self.tvec = None
        self.dst = None

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        # Publisher
        self.pub = self.create_publisher(
            Image,
            '/wrs/color/image_calibration',
            10
        )

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("Camera Pose Estimation Node Started")

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        if self.camera_info is not None:
            self.detect_checkerboard_and_publish_pose(self.cv_image)

    def detect_checkerboard_and_publish_pose(self, image):
        h, w = image.shape[:2]
        intrinsics = np.array(self.camera_info.k).reshape(3, 3)
        distortion = np.array(self.camera_info.d)

        # Undistort the image
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(intrinsics, distortion, (w,h), 1, (w,h))
        self.dst = cv2.undistort(image, intrinsics, distortion, None, newcameramtx)

        if not self.calibrated:
            checkerboard_size = (9, 6)
            ret, corners = cv2.findChessboardCorners(self.dst, checkerboard_size, None)

            if ret:
                # Object points in 3D
                objp = np.zeros((np.prod(checkerboard_size), 3), np.float32)
                objp[:, :2] = np.indices(checkerboard_size).T.reshape(-1, 2)
                objp *= 0.025  # square size in meters

                # Solve PnP
                ret, self.rvec, self.tvec = cv2.solvePnP(objp, corners, intrinsics, None)
                self.calibrated = True
                self.get_logger().info("Camera Calibrated!")
            else:
                self.get_logger().info("Checkerboard not found in the image.")

        # Publish undistorted image
        try:
            ros_image = self.bridge.cv2_to_imgmsg(self.dst, "bgr8")
            self.pub.publish(ros_image)
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        # Publish TF if calibrated
        if self.calibrated:
            ang = np.linalg.norm(self.rvec.T)
            quat = tf_transformations.quaternion_about_axis(
                ang, (self.rvec.T[0][0]/ang, self.rvec.T[0][1]/ang, self.rvec.T[0][2]/ang)
            )
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "camera"
            t.child_frame_id = "chessboard"
            t.transform.translation.x = self.tvec[0][0]
            t.transform.translation.y = self.tvec[1][0]
            t.transform.translation.z = self.tvec[2][0]
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]

            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = CameraPoseEstimation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
