#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateMerger(Node):
    def __init__(self):
        super().__init__('joint_state_merger')

        self.latest = {}   # joint_name → (position, velocity, effort)

        self.create_subscription(JointState, '/ur_internal/joint_states', self.cb, 10)
        self.create_subscription(JointState, '/robotiq_gripper/position', self.cb, 10)

        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(1/100.0, self.publish)

    def cb(self, msg):
        for i, name in enumerate(msg.name):
            pos = msg.position[i] if i < len(msg.position) else 0.0
            vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
            eff = msg.effort[i] if i < len(msg.effort) else 0.0
            self.latest[name] = (pos, vel, eff)

    def publish(self):
        if not self.latest:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.latest.keys())
        msg.position = [self.latest[n][0] for n in msg.name]
        msg.velocity = [self.latest[n][1] for n in msg.name]
        msg.effort   = [self.latest[n][2] for n in msg.name]
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(JointStateMerger())

if __name__ == '__main__':
    main()
