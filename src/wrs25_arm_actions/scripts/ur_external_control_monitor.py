#!/usr/bin/env python3
"""
ROS 2 Node for monitoring and maintaining UR External Control program
This node checks if the 'external_control' program is running on the UR dashboard
and automatically starts it if it's not running.

Author: Farshad Nozad Heravi (f.n.heravi@gmail.com)
"""

import rclpy
from rclpy.node import Node
from ur_dashboard_msgs.srv import GetProgramState, Load
from std_srvs.srv import Trigger
from enum import Enum


class MonitorState(Enum):
    """State machine for the monitor."""
    IDLE = 1
    CHECKING_STATE = 2
    LOADING_PROGRAM = 3
    PLAYING_PROGRAM = 4


class URExternalControlMonitor(Node):
    """Monitor node that ensures external_control program is always running on UR."""

    def __init__(self):
        super().__init__('ur_external_control_monitor')
        
        # Declare parameters
        self.declare_parameter('check_interval', 5.0)  # Check every 5 seconds
        self.declare_parameter('program_filename', 'external_control.urp')
        self.declare_parameter('dashboard_namespace', '/dashboard_client')
        self.declare_parameter('auto_start', True)  # Automatically start if not running
        
        # Get parameters
        self.check_interval = self.get_parameter('check_interval').value
        self.program_filename = self.get_parameter('program_filename').value
        self.dashboard_namespace = self.get_parameter('dashboard_namespace').value
        self.auto_start = self.get_parameter('auto_start').value
        
        # Service clients
        self.program_state_client = self.create_client(
            GetProgramState,
            f'{self.dashboard_namespace}/program_state'
        )
        
        self.load_program_client = self.create_client(
            Load,
            f'{self.dashboard_namespace}/load_program'
        )
        
        self.play_client = self.create_client(
            Trigger,
            f'{self.dashboard_namespace}/play'
        )
        
        # Wait for services to be available
        self.get_logger().info('Waiting for dashboard services...')
        self.wait_for_services()
        
        # State machine
        self.state = MonitorState.IDLE
        self.pending_future = None
        
        # Create timer for periodic checking
        self.timer = self.create_timer(
            self.check_interval,
            self.timer_callback
        )
        
        self.get_logger().info(
            f'UR External Control Monitor started. '
            f'Checking every {self.check_interval}s for "{self.program_filename}"'
        )
        
        # Track state to avoid spamming logs
        self.last_state = None
        self.program_running = False

    def wait_for_services(self, timeout_sec=10.0):
        """Wait for all required services to become available."""
        services = [
            (self.program_state_client, 'program_state'),
            (self.load_program_client, 'load_program'),
            (self.play_client, 'play')
        ]
        
        for client, name in services:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                self.get_logger().warn(
                    f'Service {self.dashboard_namespace}/{name} not available after {timeout_sec}s'
                )
            else:
                self.get_logger().info(f'Service {self.dashboard_namespace}/{name} is available')

    def timer_callback(self):
        """Main timer callback that handles the state machine."""
        
        # If we have a pending future, check if it's done
        if self.pending_future is not None:
            if not self.pending_future.done():
                return  # Still waiting for response
            
            # Process the response based on current state
            try:
                if self.state == MonitorState.CHECKING_STATE:
                    self.handle_state_response()
                elif self.state == MonitorState.LOADING_PROGRAM:
                    self.handle_load_response()
                elif self.state == MonitorState.PLAYING_PROGRAM:
                    self.handle_play_response()
            except Exception as e:
                self.get_logger().error(f'Error processing response: {str(e)}')
                self.state = MonitorState.IDLE
                self.pending_future = None
            
            return
        
        # If idle, start a new check
        if self.state == MonitorState.IDLE:
            self.check_program_state()

    def check_program_state(self):
        """Initiate a program state check."""
        if not self.program_state_client.service_is_ready():
            self.get_logger().warn('program_state service not ready', throttle_duration_sec=5.0)
            return
        
        request = GetProgramState.Request()
        self.pending_future = self.program_state_client.call_async(request)
        self.state = MonitorState.CHECKING_STATE

    def handle_state_response(self):
        """Handle response from program state check."""
        try:
            response = self.pending_future.result()
            self.pending_future = None
            
            if not response or not response.success:
                self.get_logger().warn(
                    f'Program state check failed: {response.answer if response else "No response"}',
                    throttle_duration_sec=5.0
                )
                self.state = MonitorState.IDLE
                return
            
            current_state = response.state.state
            program_name = response.program_name
            
            # Only log state changes to avoid spam
            if self.last_state != current_state:
                self.get_logger().info(
                    f'Program state: {current_state}, Program: {program_name}'
                )
                self.last_state = current_state
            
            # Check if external_control is running
            if current_state == 'RUNNING' and program_name == self.program_filename:
                if not self.program_running:
                    self.get_logger().info('External control program is running')
                    self.program_running = True
                self.state = MonitorState.IDLE
            else:
                # Program not running or wrong program
                if self.program_running:
                    self.get_logger().warn(
                        f'External control stopped or wrong program. State: {current_state}, Program: {program_name}'
                    )
                    self.program_running = False
                
                if self.auto_start:
                    self.get_logger().warn(
                        f'Attempting to start "{self.program_filename}"...'
                    )
                    self.load_program()
                else:
                    self.get_logger().warn(
                        f'External control is {current_state}. Auto-start is disabled.',
                        throttle_duration_sec=10.0
                    )
                    self.state = MonitorState.IDLE
                    
        except Exception as e:
            self.get_logger().error(f'Exception handling state response: {str(e)}')
            self.state = MonitorState.IDLE
            self.pending_future = None

    def load_program(self):
        """Initiate loading the external control program."""
        if not self.load_program_client.service_is_ready():
            self.get_logger().error('load_program service not ready')
            self.state = MonitorState.IDLE
            return
        
        load_request = Load.Request()
        load_request.filename = self.program_filename
        
        self.get_logger().info(f'Loading program: {self.program_filename}')
        self.pending_future = self.load_program_client.call_async(load_request)
        self.state = MonitorState.LOADING_PROGRAM

    def handle_load_response(self):
        """Handle response from load program request."""
        try:
            response = self.pending_future.result()
            self.pending_future = None
            
            if not response or not response.success:
                self.get_logger().error(
                    f'Failed to load program: {response.answer if response else "No response"}'
                )
                self.state = MonitorState.IDLE
                return
            
            self.get_logger().info(f'Program loaded: {response.answer}')
            self.play_program()
            
        except Exception as e:
            self.get_logger().error(f'Exception handling load response: {str(e)}')
            self.state = MonitorState.IDLE
            self.pending_future = None

    def play_program(self):
        """Initiate playing the loaded program."""
        if not self.play_client.service_is_ready():
            self.get_logger().error('play service not ready')
            self.state = MonitorState.IDLE
            return
        
        play_request = Trigger.Request()
        
        self.get_logger().info('Starting program...')
        self.pending_future = self.play_client.call_async(play_request)
        self.state = MonitorState.PLAYING_PROGRAM

    def handle_play_response(self):
        """Handle response from play program request."""
        try:
            response = self.pending_future.result()
            self.pending_future = None
            
            if not response or not response.success:
                self.get_logger().error(
                    f'Failed to play program: {response.message if response else "No response"}'
                )
            else:
                self.get_logger().info(f'Program started successfully: {response.message}')
                self.program_running = True
            
            self.state = MonitorState.IDLE
            
        except Exception as e:
            self.get_logger().error(f'Exception handling play response: {str(e)}')
            self.state = MonitorState.IDLE
            self.pending_future = None


def main(args=None):
    """Main function to start the monitor node."""
    rclpy.init(args=args)
    
    try:
        monitor = URExternalControlMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Exception in monitor node: {str(e)}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

