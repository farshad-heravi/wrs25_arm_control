#!/usr/bin/env python3
"""
Joint state publisher for gripper joint when using real hardware.
Subscribes to joint states from UR driver and adds the gripper joint state.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading


class GripperJointStatePublisher(Node):
    def __init__(self):
        super().__init__('gripper_joint_state_publisher')
        
        # Gripper joint name
        self.gripper_joint_name = 'ur5_robotiq_85_left_knuckle_joint'
        
        # Initial position (0.0 = open)
        self.gripper_position = 0.0
        
        # Lock for thread-safe access
        self.lock = threading.Lock()
        
        # Store the latest joint state from UR driver
        self.latest_joint_state = None
        
        # Create publisher for merged joint states
        self.publisher_ = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Subscribe to joint states from UR driver
        # We check if gripper joint already exists to avoid feedback loops
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.get_logger().info(f'GripperJointStatePublisher started. Adding joint: {self.gripper_joint_name}')
    
    def joint_state_callback(self, msg):
        """Receive joint state from UR driver and republish with gripper joint added"""
        with self.lock:
            # Check if this message already has the gripper joint (avoid feedback)
            if self.gripper_joint_name in msg.name:
                # This message already has gripper, don't republish to avoid loop
                return
            
            # Create new joint state message
            joint_state = JointState()
            joint_state.header = msg.header
            joint_state.name = list(msg.name)
            joint_state.position = list(msg.position)
            
            # Copy velocity and effort arrays if they exist
            if len(msg.velocity) > 0:
                joint_state.velocity = list(msg.velocity)
            else:
                joint_state.velocity = []
            
            if len(msg.effort) > 0:
                joint_state.effort = list(msg.effort)
            else:
                joint_state.effort = []
            
            # Add gripper joint
            joint_state.name.append(self.gripper_joint_name)
            joint_state.position.append(self.gripper_position)
            
            # Add velocity and effort for gripper if arrays exist
            if len(joint_state.velocity) > 0:
                joint_state.velocity.append(0.0)
            if len(joint_state.effort) > 0:
                joint_state.effort.append(0.0)
            
            # Publish merged joint state
            self.publisher_.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    node = GripperJointStatePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

