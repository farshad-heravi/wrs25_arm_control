#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
import tf2_ros
from tf_transformations import euler_matrix, quaternion_from_matrix, quaternion_matrix
import cv2
import math

class BottlePoseNode(Node):
    def __init__(self):
        super().__init__('bottle_pose_node')

        # Subscribers
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/bottle_pixel_pose', self.pose_callback, 10)


        # Publisher
        self.pub_pose = self.create_publisher(PoseStamped, '/bottle_pose', 10)

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Camera info
        self.camera_info = None

        # Chessboard to robot transformation (fixed/calibrated)
        self.T_ch_r = self.compute_chessboard_to_robot()

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
        pos_ch_r = np.array([0.3378, -0.5781, -0.1595])
        # euler_ch_r = np.array([math.pi, 0.0, 0.0])
        # R_ch_r, _ = cv2.Rodrigues(R_ch_r[:3,:3])
        quaternion = np.array([0.0026, -0.7053, 0.7089, -0.0002]) #wxyz
        R_ch_r = quaternion_matrix(quaternion)
        R_ch_r = R_ch_r[:3,:3]
        T_ch_r = np.eye(4)
        T_ch_r[:3, :3] = R_ch_r
        T_ch_r[:3, 3] = pos_ch_r

        # publish chessboard world tf
        world_c_quat = quaternion_from_matrix(T_ch_r)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "ur5_base"
        t.child_frame_id = "chessboard"
        t.transform.translation.x = T_ch_r[3,0]
        t.transform.translation.y = T_ch_r[3,1]
        t.transform.translation.z = T_ch_r[3,2]
        t.transform.rotation.x = world_c_quat[1]
        t.transform.rotation.y = world_c_quat[2]
        t.transform.rotation.z = world_c_quat[3]
        t.transform.rotation.w = world_c_quat[0]
        self.tf_broadcaster.sendTransform(t)

        return T_ch_r

    def compute_bottle_pose(self, pose_msg):
        # Step 1: Bottle pixel coordinates
        cx = pose_msg.pose.position.x
        cy = pose_msg.pose.position.y
        rotation = pose_msg.pose.position.z  # bottle orientation in image plane

        # Step 2: Convert pixel to camera coordinates
        K = np.array(self.camera_info.k).reshape(3,3)
        pixel_coords = np.array([cx, cy, 1.0])
        point_cam = np.linalg.inv(K) @ pixel_coords

        # Step 3: Bottle -> camera homogeneous transformation
        T_o_c = euler_matrix(rotation, 0, 0, 'ryzx')
        T_o_c[:3, 3] = point_cam

        # Step 4: Lookup chessboard in camera frame (TF listener)
        try:
            t = self.tf_buffer.lookup_transform(
                'camera',
                'chessboard',
                rclpy.time.Time(),  # Lookup at the latest available time
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            trans = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
            quat_msg = t.transform.rotation
            quat = [quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w]
            T_ch_c = quaternion_matrix(quat)
            T_ch_c[:3, 3] = trans
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn("Chessboard TF not found")
            return

        

        # Step 5: Compute bottle -> robot
        T_o_r = self.T_ch_r @ np.linalg.inv(T_ch_c) @ T_o_c

        quat_final = quaternion_from_matrix(T_o_r)

        # Step 6: Broadcast TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "bottle_frame"
        t.transform.translation.x = T_o_r[3,0]
        t.transform.translation.y = T_o_r[3,1]
        t.transform.translation.z = T_o_r[3,2]
        t.transform.rotation.x = quat_final[1]
        t.transform.rotation.y = quat_final[2]
        t.transform.rotation.z = quat_final[3]
        t.transform.rotation.w = quat_final[0]
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

        self.pub_pose.publish(pose_out)
        self.get_logger().info(f"Bottle pose published: {pose_out.pose.position}")


        # publish world chessboard tf

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
