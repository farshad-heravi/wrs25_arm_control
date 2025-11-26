#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
import tf2_ros
from tf_transformations import euler_matrix, quaternion_from_matrix, quaternion_matrix, quaternion_from_euler, quaternion_multiply
import cv2
import math
from cam_calibration.action import BottleCalibration

class BottlePoseNode(Node):
    def __init__(self):
        super().__init__('bottle_pose_node')

        # Subscribers
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/bottle_pixel_pose', self.pose_callback, 10)


        # Publisher
        self.pub_pose = self.create_publisher(PoseStamped, '/bottle_pose', 10)

        # Action Server
        self._action_server = ActionServer(
            self,
            BottleCalibration,
            'bottle_calibration_action',
            self.execute_calibration_callback
        )
        self.get_logger().info('Bottle calibration action server initialized')

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Camera info
        self.camera_info = None

        # Chessboard to robot transformation (fixed/calibrated)
        self.T_ch_r = self.compute_chessboard_to_robot()

        #
        #self.timer = self.create_timer(1.0/200, self.timer_callback)

    def timer_callback(self):
        self.compute_chessboard_to_robot()

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def pose_callback(self, msg: PoseStamped):
        if self.camera_info is None:
            self.get_logger().warn("Camera info not yet received")
            return
            
        self.compute_bottle_pose(msg)

    def compute_chessboard_to_robot(self):
        """
        Returns 4x4 homogeneous transformation from chessboard to robot base.
        """
        pos_ch_r = np.array([0.3818, -0.7384, -0.1602])
        # euler_ch_r = np.array([math.pi, 0.0, 0.0])
        # R_ch_r, _ = cv2.Rodrigues(R_ch_r[:3,:3])
        quaternion = np.array([0.0012, 0.7028, 0.7114, 0.0031]) #wxyz
        q_180 = quaternion_from_euler(np.pi/2, 0, 0)
        q_rot = quaternion_multiply(quaternion, q_180)
        R_ch_r = quaternion_matrix(q_rot)
        R_ch_r = R_ch_r[:3,:3]
        T_ch_r = np.eye(4)
        T_ch_r[:3, :3] = R_ch_r
        T_ch_r[:3, 3] = pos_ch_r

        # publish chessboard world tf
        world_c_quat = quaternion_from_matrix(T_ch_r)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world" # was world
        t.child_frame_id = "chessboard"
        t.transform.translation.x = T_ch_r[0,3]
        t.transform.translation.y = T_ch_r[1,3]
        t.transform.translation.z = T_ch_r[2,3]
        t.transform.rotation.x = world_c_quat[0]
        t.transform.rotation.y = world_c_quat[1]
        t.transform.rotation.z = world_c_quat[2]
        t.transform.rotation.w = world_c_quat[3]
        # self.tf_broadcaster.sendTransform(t)
        self.static_broadcaster.sendTransform(t)
        self.get_logger().info("Chessboard to robot TF published!")

        return T_ch_r

    def compute_bottle_pose(self, pose_msg):
        # Step 1: Bottle pixel coordinates
        cx = pose_msg.pose.position.x
        cy = pose_msg.pose.position.y
        rotation = pose_msg.pose.position.z  # bottle orientation in image plane

        # Step 1.5: Undistort the points
        K = np.array(self.camera_info.k).reshape(3, 3)
        distortion = np.array(self.camera_info.d)

        # Undistort the image
        udP = cv2.undistortPoints(np.array([[cx, cy]]), K, distortion, P=K)
        
        # Step 2: Convert pixel to camera coordinates
        udP = udP[0]
        pixel_coords = np.array([udP[0,0], udP[0,1], 1.0]).T
        point_cam = np.linalg.inv(K) @ pixel_coords

        # Step 3: Bottle -> camera homogeneous transformation
        T_o_c = euler_matrix(rotation, 0, 0, 'ryzx')
        T_o_c[:3, 3] = point_cam

        # Step 4: Lookup chessboard in camera frame (TF listener)
        try:
            t = self.tf_buffer.lookup_transform(
                "camera",
                "chessboard",
                rclpy.time.Time(),  # Lookup at the latest available time
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            trans = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
            quat_msg = t.transform.rotation
            quat = [quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w]
            T_ch_c = quaternion_matrix(quat)
            T_ch_c[:3, 3] = trans
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Chessboard TF not found: {str(e)}")
            return None

        

        # Step 5: Compute bottle -> robot
        # T_o_r = self.T_ch_r @ np.linalg.inv(T_ch_c) @ T_o_c
        # quat_final = quaternion_from_matrix(T_o_r)

        ###################### 
        T_r_ch = np.linalg.inv(self.T_ch_r)
        # other test...
        rotation_matrix = T_ch_c[:3, :3]
        point_in_camera_coordinate = point_cam
        translation_vector = T_ch_c[:3, 3]
        C = - np.linalg.inv(rotation_matrix) @ translation_vector
        point_in_world_coordinate = np.linalg.inv(rotation_matrix) @ (point_in_camera_coordinate - translation_vector)
        plane_equation = np.array([0, 0, 1, -0.21]) # a, b, c, d
        t = (plane_equation[-1] - np.dot(plane_equation[0:3], C)) / np.dot(plane_equation[0:3], (point_in_world_coordinate - C))
        approximated_world_coord = C + t * (point_in_world_coordinate - C)
        print(approximated_world_coord)
        print(np.linalg.inv(T_r_ch[:3,:3]) @ -T_r_ch[:3,3] + np.linalg.inv(T_r_ch[:3,:3]) @ approximated_world_coord)
        ####################################################################
        ####################################################################                
        realz = T_ch_c[:3, 3]

        T_o_ch = np.linalg.inv(T_ch_c) @ T_o_c


        T_o_r = euler_matrix(rotation - np.pi/2, 0, np.pi, 'rzyx')

        T_o_r[:3,3] = np.linalg.inv(T_r_ch[:3,:3]) @ -T_r_ch[:3,3] + np.linalg.inv(T_r_ch[:3,:3]) @ approximated_world_coord

        #T_o_r = T_r_ch @ np.linalg.inv(T_o_ch).T

        quat_final = quaternion_from_matrix(T_o_r)

        #########################

        # Step 6: Broadcast TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "bottle_frame"
        t.transform.translation.x = T_o_r[0,3]
        t.transform.translation.y = T_o_r[1,3]
        t.transform.translation.z = T_o_r[2,3]
        t.transform.rotation.x = quat_final[0]
        t.transform.rotation.y = quat_final[1]
        t.transform.rotation.z = quat_final[2]
        t.transform.rotation.w = quat_final[3]
        self.tf_broadcaster.sendTransform(t)

        # Step 7: Publish PoseStamped
        pose_out = PoseStamped()
        pose_out.header.stamp = self.get_clock().now().to_msg()
        pose_out.header.frame_id = 'world'
        pose_out.pose.position.x = T_o_r[0,3]
        pose_out.pose.position.y = T_o_r[1,3]
        pose_out.pose.position.z = T_o_r[2,3]
        q = quaternion_from_matrix(T_o_r)
        pose_out.pose.orientation.x = q[0]
        pose_out.pose.orientation.y = q[1]
        pose_out.pose.orientation.z = q[2]
        pose_out.pose.orientation.w = q[3]

        # Publish to topic
        self.pub_pose.publish(pose_out)
        self.get_logger().info(f"Bottle pose published: {pose_out.pose.position}")

        # Return the computed pose
        return pose_out

    def execute_calibration_callback(self, goal_handle):
        """
        Action server callback for bottle calibration.
        Receives pixel pose and returns computed bottle pose.
        """
        self.get_logger().info('Executing bottle calibration action...')
        
        # Send feedback
        feedback_msg = BottleCalibration.Feedback()
        feedback_msg.status = 'Processing bottle pixel pose'
        goal_handle.publish_feedback(feedback_msg)
        
        # Check if camera info is available
        if self.camera_info is None:
            self.get_logger().error("Camera info not yet received")
            goal_handle.abort()
            result = BottleCalibration.Result()
            result.success = False
            result.message = "Camera info not available"
            result.bottle_pose = PoseStamped()
            return result
        
        # Get the pixel pose from the goal
        pixel_pose = goal_handle.request.pixel_pose
        
        # Compute bottle pose
        feedback_msg.status = 'Computing bottle pose in robot frame'
        goal_handle.publish_feedback(feedback_msg)
        
        bottle_pose = self.compute_bottle_pose(pixel_pose)
        
        # Prepare result
        result = BottleCalibration.Result()
        
        if bottle_pose is None:
            self.get_logger().error("Failed to compute bottle pose")
            goal_handle.abort()
            result.success = False
            result.message = "Failed to compute bottle pose - TF lookup failed"
            result.bottle_pose = PoseStamped()
        else:
            self.get_logger().info(f"Bottle calibration completed successfully")
            goal_handle.succeed()
            result.success = True
            result.message = "Bottle pose computed successfully"
            result.bottle_pose = bottle_pose
        
        return result

    '''
    def computeBottlePose(self, rect, idx):
        #print(rect)
        # T_ch_r: Homogeneous transformation from robot to checkerboard (of checkerboard w.r.t. robot)
        # T_ch_c: Homogeneous transformation from camera to checkerboard (of checkerboard w.r.t. camera)
        # T_o_c: Homogeneous transformation from camera to object (of object w.r.t. camera)
        # T_o_ch: Homogeneous transformation from checkerboard to object (of object w.r.t. checkerboard)
        # T_o_r: Homogeneous transformation from robot to object (of object w.r.t. robot)
        try:
            (trans,quat) = self.tf_listener.lookupTransform('/camera', '/chessboard', rclpy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rclpy.WARN("Occhio!")

        rotation_matrix = np.array(quaternion_matrix(quat))

        # Create a 4x4 transformation matrix in homogeneous coordinates
        T_ch_c = np.eye(4)  # Identity matrix

        # Set the upper-left 3x3 block to the rotation matrix
        T_ch_c[:3, :3] = rotation_matrix[:3, :3]

        # Set the right-most column to the translation vector
        T_ch_c[:3, 3] = trans
        print(T_ch_c)
        
        # Compute bottle pose...
        
        T_o_c = np.eye(4)  # Identity matrix

        Bottle_center = rect[0]


        pixel_coordinates = np.array([Bottle_center[0] + 620, Bottle_center[1] + 150, 1]).T; 

        K_v = np.array(self.camera_info.K)
        K_v[:-1] = K_v[:-1] 
        K = K_v.reshape(3,3)

        # point_3d
        point_3d = np.linalg.inv(K) @ pixel_coordinates
        rotation = rect[2]
        
        T_o_c = euler_matrix(rotation, 0, 0, 'ryzx')
        #R_o_c, _  = cv2.Rodrigues(np.array([0, 0, rotation]))
        # rotation matrix should be the rotation of the chessboard
        #### Transformation from the Bottle frame to the camera frame
        # Set the right-most column to the translation vector
        T_o_c[:3, 3] = point_3d


        br1 = tf.TransformBroadcaster()
        br2 = tf.TransformBroadcaster()
        br3 = tf.TransformBroadcaster()

        # added by mlahoud
        pos_ch_r = np.array([-0.11417, -0.61179, -0.074]) # [m]
        orn_ch_r = np.array([3.1415, -0.0, 0.0]) # [rad]
        R_ch_r, _ = cv2.Rodrigues(orn_ch_r)
        # now in homogeneous matrix form
        T_ch_r = np.array([[R_ch_r[0,0], R_ch_r[0,1], R_ch_r[0,2], pos_ch_r[0]],
                           [R_ch_r[1,0], R_ch_r[1,1], R_ch_r[1,2], pos_ch_r[1]],
                           [R_ch_r[2,0], R_ch_r[2,1], R_ch_r[2,2], pos_ch_r[2]],
                           [0,0,0,1]])
        #print(T_ch_r)
        T_r_ch = np.linalg.inv( T_ch_r)  
        ####################################################################
        ##########################################################
        # other test...
        rotation_matrix = T_ch_c[:3, :3]
        point_in_camera_coordinate = point_3d
        translation_vector = T_ch_c[:3, 3]
        C = - np.linalg.inv(rotation_matrix) @ translation_vector
        point_in_world_coordinate = np.linalg.inv(rotation_matrix) @ (point_in_camera_coordinate - translation_vector)
        plane_equation = np.array([0, 0, 1, 0]) # a, b, c, d
        t = (plane_equation[-1] - np.dot(plane_equation[0:3], C)) / np.dot(plane_equation[0:3], (point_in_world_coordinate - C))
        approximated_world_coord = C + t * (point_in_world_coordinate - C)
        print(approximated_world_coord)
        print(np.linalg.inv(T_r_ch[:3,:3]) @ -T_r_ch[:3,3] + np.linalg.inv(T_r_ch[:3,:3]) @ approximated_world_coord)
        ####################################################################
        ####################################################################                
        realz = T_ch_c[:3, 3]

        T_o_ch = np.linalg.inv(T_ch_c) @ T_o_c


        br2.sendTransform((T_r_ch[0][3], T_r_ch[1][3], T_r_ch[2][3]),
                        tf.transformations.quaternion_from_matrix(T_r_ch),
                        rclpy.Time.now(),
                        '/world',
                        "/chessboard")  

        T_o_r = euler_matrix(-rotation - np.pi/2, 0, np.pi, 'rzyx')
        Rotation_AR = euler_matrix(rotation, 0, np.pi, 'ryxz')
        quaternion_AR = tf.transformations.quaternion_from_matrix(Rotation_AR)
        print(quaternion_AR)
        T_o_r[:3,3] = np.linalg.inv(T_r_ch[:3,:3]) @ -T_r_ch[:3,3] + np.linalg.inv(T_r_ch[:3,:3]) @ approximated_world_coord
        [0.652956, -0.0739962, -0.282485]
        trans_AR = np.array([-T_o_r[1][3], T_o_r[2][3], T_o_r[0][3]])

        #T_o_r = T_r_ch @ np.linalg.inv(T_o_ch).T

        quaternion_T_o_r = tf.transformations.quaternion_from_matrix(T_o_r)

        frame_id = f"bottle_pose{i}"

        br3.sendTransform((T_o_r[0][3], T_o_r[1][3], T_o_r[2][3]),
                        tf.transformations.quaternion_from_matrix(T_o_r),
                        rclpy.Time.now(),
                        '/bottle_pose',
                        "/world")  
        # print("T_o_r")
        # print(T_o_r) 
        # print(quaternion_T_o_r)
        # Do prepare for the publication of the bottle_pose
    
        try:
            (trans,quat) = self.tf_listener2.lookupTransform('/world', frame_id , rclpy.Time(0))
            # header
            self.bottle_pose.header.frame_id = "/world"
            self.bottle_pose.header.stamp = rclpy.Time.now()#
            # pose
            # position
            self.bottle_pose.pose.position.x = trans_AR[0]
            self.bottle_pose.pose.position.y = trans_AR[1]
            self.bottle_pose.pose.position.z = trans_AR[2] # half height of the physical Bottle
            # orientation
            self.bottle_pose.pose.orientation.x = quaternion_AR[0]
            self.bottle_pose.pose.orientation.y = quaternion_AR[1]
            self.bottle_pose.pose.orientation.z = quaternion_AR[2]
            self.bottle_pose.pose.orientation.w = quaternion_AR[3]
            # publish
            self.pub_pose.publish(self.bottle_pose)
            print("publishing Bottle frame wrt robot frame to hololens...")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rclpy.WARN("Occhio!")
    '''    

def main(args=None):
    rclpy.init(args=args)
    node = BottlePoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
