#!/usr/bin/env python3
"""
ROS 2 Action Server for Robotiq Gripper Control
Author: Farshad Nozad Heravi (f.n.heravi@gmail.com)
"""

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState

from wrs25_arm_actions.action import RobotiqGripperControl
import robotiq_gripper
from control_msgs.action import GripperCommand


class RobotiqGripperActionServer(Node):
    """Action server for controlling Robotiq gripper hardware."""

    def __init__(self):
        super().__init__('robotiq_gripper_action_server')
        
        # Declare parameters with defaults
        self.declare_parameter('default_ip', '192.168.1.101')
        self.declare_parameter('default_port', 63352)
        self.declare_parameter('auto_connect', True)
        self.declare_parameter('fake_joint_state', False) # publishes 0; for testing without real hardware
        
        # Get parameters
        self.default_ip = self.get_parameter('default_ip').value
        self.default_port = self.get_parameter('default_port').value
        auto_connect = self.get_parameter('auto_connect').value
        fake_joint_state = self.get_parameter('fake_joint_state').value
        self.fake_joint_state = fake_joint_state
        self.fake_joint_state_position = 0.0

        # Initialize gripper object
        self.gripper = None
        self.connected = False
        
        # Create publisher for gripper position (raw Robotiq position 0-255)
        self.position_publisher = self.create_publisher(
            JointState,
            '/robotiq_gripper/position',
            10
        )
        
        # Create timer to periodically publish gripper position
        self.publish_timer = self.create_timer(0.1, self.publish_position_callback)
        
        # Create action server - for real hardware
        if not self.fake_joint_state:
            self._action_server = ActionServer(
                self,
                RobotiqGripperControl,
                'robotiq_gripper_control',
                execute_callback=self.execute_callback,
                goal_callback=self.goal_callback,
                cancel_callback=self.cancel_callback,
                callback_group=ReentrantCallbackGroup()
            )
        else:
            self._action_server_moveit = ActionServer(
                self,
                GripperCommand,
                '/hand_controller/gripper_command',
                execute_callback=self.execute_callback_moveit,
                goal_callback=self.goal_callback_moveit,
                cancel_callback=self.cancel_callback_moveit,
                callback_group=ReentrantCallbackGroup()
            )
        
        self.get_logger().info('Robotiq Gripper Action Server initialized')
        if not self.fake_joint_state:
            self.get_logger().info(f'Default IP: {self.default_ip}, Port: {self.default_port}')
        
        # Auto-connect if requested
        if auto_connect and not self.fake_joint_state:
            try:
                self._connect_gripper(self.default_ip, self.default_port)
            except Exception as e:
                self.get_logger().error(f'Auto-connect failed: {e}')

    def _connect_gripper(self, ip: str, port: int):
        """Connect to the gripper."""
        if self.gripper is None:
            self.gripper = robotiq_gripper.RobotiqGripper()
        
        if not self.connected:
            self.get_logger().info(f'Connecting to gripper at {ip}:{port}...')
            self.gripper.connect(ip, port)
            self.get_logger().info('Activating gripper...')
            self.gripper.activate()
            self.connected = True
            self.get_logger().info('Gripper connected and activated')

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def publish_position_callback(self):
        """Periodically publish current gripper position."""
        if self.fake_joint_state:
            msg = JointState()
            msg.name = ['ur5_finger_joint']
            msg.position = [self.fake_joint_state_position]
            msg.velocity = [0.0]
            msg.effort = [0.0]
            self.position_publisher.publish(msg)
            return
        elif self.gripper is not None and self.connected:
            try:
                # Get current position from gripper
                current_pos = self.gripper.get_current_position()

                # scale current position to 0-0.7 for 2f140 gripper
                current_pos = current_pos / 255.0 * 0.7
                
                # Publish position
                msg = JointState()
                msg.name = ['ur5_finger_joint']
                msg.position = [float(current_pos)]
                msg.velocity = [0.0]
                msg.effort = [0.0]
                self.position_publisher.publish(msg)
            except Exception as e:
                # Silently fail to avoid spamming logs
                pass

    def execute_callback(self, goal_handle):
        """Execute the action."""
        self.get_logger().info('Executing goal...')
        
        request = goal_handle.request
        feedback_msg = RobotiqGripperControl.Feedback()
        result = RobotiqGripperControl.Result()
        
        # Get connection parameters
        ip = request.ip_address if request.ip_address else self.default_ip
        port = request.port if request.port > 0 else self.default_port
        
        try:
            # Connect to gripper if not already connected
            if not self.fake_joint_state:
                self._connect_gripper(ip, port)
            
            # Determine target position based on command
            if request.command.lower() == 'open':
                target_pos = self.gripper.get_open_position()
                speed = request.speed if request.speed > 0 else 255
                force = request.force if request.force > 0 else 255
                self.get_logger().info('Opening gripper...')
                
            elif request.command.lower() == 'close':
                target_pos = self.gripper.get_closed_position()
                speed = request.speed if request.speed > 0 else 255
                force = request.force if request.force > 0 else 255
                self.get_logger().info('Closing gripper...')
                
            elif request.command.lower() == 'move':
                target_pos = request.position
                speed = request.speed if request.speed > 0 else 255
                force = request.force if request.force > 0 else 255
                self.get_logger().info(f'Moving gripper to position {target_pos}...')
                
            else:
                result.success = False
                result.message = f'Unknown command: {request.command}'
                goal_handle.abort()
                return result
            
            # Move the gripper and wait for completion
            final_pos, obj_status = self.gripper.move_and_wait_for_pos(
                target_pos, speed, force
            )
            
            # Send final feedback
            feedback_msg.current_position = final_pos
            feedback_msg.is_moving = False
            goal_handle.publish_feedback(feedback_msg)
            
            # Set result
            result.success = True
            result.final_position = final_pos
            result.message = f'Gripper {request.command} completed. Final position: {final_pos}, Status: {obj_status.name}'
            
            self.get_logger().info(result.message)
            goal_handle.succeed()
            
        except Exception as e:
            result.success = False
            result.message = f'Error: {str(e)}'
            result.final_position = -1
            self.get_logger().error(result.message)
            goal_handle.abort()
        
        return result

    def destroy_node(self):
        """Clean up before destroying the node."""
        if self.gripper is not None and self.connected:
            try:
                self.gripper.disconnect()
                self.get_logger().info('Gripper disconnected')
            except Exception as e:
                self.get_logger().error(f'Error disconnecting gripper: {e}')
        super().destroy_node()

    def execute_callback_moveit(self, goal_handle):
        """Execute the action."""
        self.get_logger().info('Executing goal...')
        
        request = goal_handle.request
        desired_position = request.command.position
        self.get_logger().info(f'requested position: {desired_position}')

        goal_handle.succeed()
        
        result = GripperCommand.Result()
        result.reached_goal = True
        result.position = desired_position

        if self.fake_joint_state:
            self.fake_joint_state_position = desired_position
        else:
            self.gripper.move_and_wait_for_pos(
                desired_position, 255, 255
            )

        self.get_logger().info('Goal succeeded')
        return result

    def goal_callback_moveit(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')

        return GoalResponse.ACCEPT

    def cancel_callback_moveit(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')

def main(args=None):
    rclpy.init(args=args)
    
    server = RobotiqGripperActionServer()
    executor = MultiThreadedExecutor()
    
    try:
        rclpy.spin(server, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

