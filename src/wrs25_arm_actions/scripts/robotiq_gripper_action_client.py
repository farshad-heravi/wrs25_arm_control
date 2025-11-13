#!/usr/bin/env python3
"""
ROS 2 Action Client for Robotiq Gripper Control
Author: Farshad Nozad Heravi (f.n.heravi@gmail.com)
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from wrs25_arm_actions.action import RobotiqGripperControl


class RobotiqGripperActionClient(Node):
    """Action client for testing Robotiq gripper control."""

    def __init__(self):
        super().__init__('robotiq_gripper_action_client')
        
        # Declare parameters
        self.declare_parameter('command', 'open')  # open, close, or move
        self.declare_parameter('position', 0)       # 0-255
        self.declare_parameter('speed', 255)        # 0-255
        self.declare_parameter('force', 255)        # 0-255
        self.declare_parameter('ip_address', '')    # Empty string uses server default
        self.declare_parameter('port', 0)           # 0 uses server default
        
        self._action_client = ActionClient(
            self,
            RobotiqGripperControl,
            'robotiq_gripper_control'
        )

    def send_goal(self):
        """Send a goal to the action server."""
        # Get parameters
        command = self.get_parameter('command').value
        position = self.get_parameter('position').value
        speed = self.get_parameter('speed').value
        force = self.get_parameter('force').value
        ip_address = self.get_parameter('ip_address').value
        port = self.get_parameter('port').value
        
        # Wait for action server
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        
        # Create goal message
        goal_msg = RobotiqGripperControl.Goal()
        goal_msg.command = command
        goal_msg.position = position
        goal_msg.speed = speed
        goal_msg.force = force
        goal_msg.ip_address = ip_address
        goal_msg.port = port
        
        self.get_logger().info(f'Sending goal: {command}')
        if command == 'move':
            self.get_logger().info(f'Position: {position}, Speed: {speed}, Force: {force}')
        
        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response from server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            return
        
        self.get_logger().info('Goal accepted by server, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action server."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback - Position: {feedback.current_position}, Moving: {feedback.is_moving}'
        )

    def get_result_callback(self, future):
        """Handle the final result from the action server."""
        result = future.result().result
        
        if result.success:
            self.get_logger().info(f'Success! {result.message}')
        else:
            self.get_logger().error(f'Failed: {result.message}')
        
        # Shutdown after receiving result
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    client = RobotiqGripperActionClient()
    client.send_goal()
    
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()

