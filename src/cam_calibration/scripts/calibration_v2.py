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
import yaml
import os

flag_imshow = False
pwd = os.path.dirname(os.path.abspath(__file__))
CALIB_PATH = os.path.join(pwd, "calibration.yaml")

class CameraPoseEstimation(Node):
    def __init__(self):
        super().__init__('camera_pose_estimation_node')

        # Parameter to force new calibration
        self.declare_parameter("force_calibration", False)
        self.force_calibration = self.get_parameter("force_calibration").value

        self.bridge = CvBridge()
        self.camera_info = None
        self.calibrated = False
        self.rvec = None
        self.tvec = None
        self.dst = None

        # Try loading existing calibration
        self.try_load_calibration()

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
        self.camera_frame = None
        self.camera_frame_timer = self.create_timer(1.0/200, self.camera_frame_timer_callback)

        self.get_logger().info("Camera Pose Estimation Node Started")

    def camera_frame_timer_callback(self):
        if self.camera_frame is not None:
            self.camera_frame.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(self.camera_frame)

    # -------------------- YAML LOAD & SAVE --------------------

    def try_load_calibration(self):
        """Loads rvec/tvec from YAML unless force_calibration=True."""
        if self.force_calibration:
            self.get_logger().info("Force calibration requested — ignoring saved calibration.")
            return

        if os.path.exists(CALIB_PATH):
            try:
                with open(CALIB_PATH, "r") as f:
                    data = yaml.safe_load(f)

                self.tvec = np.array(data["tvec"], dtype=np.float32).reshape(3, 1)
                self.rvec = np.array(data["rvec"], dtype=np.float32).reshape(3, 1)

                self.calibrated = True
                self.get_logger().info(f"Loaded calibration from {CALIB_PATH}")

            except Exception as e:
                self.get_logger().error(f"Failed to load YAML calibration: {e}")

    def save_calibration(self):
        """Saves rvec/tvec in YAML file."""
        data = {
            "tvec": self.tvec.reshape(-1).tolist(),
            "rvec": self.rvec.reshape(-1).tolist()
        }
        try:
            with open(CALIB_PATH, "w") as f:
                yaml.safe_dump(data, f, sort_keys=False)
            self.get_logger().info(f"Calibration saved to {CALIB_PATH}")
        except Exception as e:
            self.get_logger().error(f"Failed to save YAML calibration: {e}")

    # -------------------- ROS CALLBACKS --------------------

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

    # -------------------- CHESSBOARD + TF --------------------

    def detect_checkerboard_and_publish_pose(self, image):
        h, w = image.shape[:2]
        intrinsics = np.array(self.camera_info.k).reshape(3, 3)
        distortion = np.array(self.camera_info.d)

        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(intrinsics, distortion, (w, h), 1, (w, h))
        self.dst = cv2.undistort(image, intrinsics, distortion, None, newcameramtx)

        if not self.calibrated:
            checkerboard_size = (9, 6)
            ret, corners = cv2.findChessboardCorners(self.dst, checkerboard_size, None)

            if ret:
                objp = np.zeros((np.prod(checkerboard_size), 3), np.float32)
                objp[:, :2] = np.indices(checkerboard_size).T.reshape(-1, 2)
                objp *= 0.025

                ret, rvec, tvec = cv2.solvePnP(objp, corners, intrinsics, None)
                axes_im = cv2.drawFrameAxes(self.dst, intrinsics, None, rvec, tvec, 0.025, 3)
                
                if flag_imshow:
                    cv2.imshow("Axes", axes_im)
                    cv2.waitKey()
                    
                if ret:
                    self.rvec = rvec
                    self.tvec = tvec
                    self.calibrated = True
                    self.save_calibration()
                    self.get_logger().info("Camera successfully calibrated!")
            else:
                self.get_logger().info("Checkerboard not found.")

        # publish image
        try:
            ros_image = self.bridge.cv2_to_imgmsg(self.dst, "bgr8")
            self.pub.publish(ros_image)
        except CvBridgeError as e:
            return

        if self.calibrated:
            ang = np.linalg.norm(self.rvec)
            axis = self.rvec.reshape(3) / ang
            quat = tf_transformations.quaternion_about_axis(ang, axis)


            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "camera"
            t.child_frame_id = "chessboard"

            t.transform.translation.x = float(self.tvec[0])
            t.transform.translation.y = float(self.tvec[1])
            t.transform.translation.z = float(self.tvec[2])

            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
            self.camera_frame = t

            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = CameraPoseEstimation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
