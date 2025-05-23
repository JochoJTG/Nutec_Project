#!/usr/bin/env python3.10

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import  Vector3
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import math

class reference(Node):
    def __init__(self):
        super().__init__('reference')



        self.publisher_ = self.create_publisher(PoseStamped, '/mecanum_bot/reference_pose', 10)
        self.timer = self.create_timer(0.1, self.publish_reference)  # 10 Hz

        self.u_length = 10.0  # meters
        self.u_width = 2.3   # meters
        self.speed = 0.04     # m per step
        self.step = 0
        self.forward = True
        self.traj = self.generate_u_trajectory()

    def generate_u_trajectory(self):
        traj = []

        # Left vertical part (facing forward)
        for y in range(0, int(self.u_length / self.speed)):
            traj.append((0.0, y * self.speed, 0.0))  # yaw = 0

        # Bottom horizontal part (turning)
        for x in range(1, int(self.u_width / self.speed)):
            traj.append((x * self.speed, self.u_length, math.pi / 2))  # turn right

        # Right vertical part (facing backward)
        for y in range(int(self.u_length / self.speed) - 1, -1, -1):
            traj.append((self.u_width, y * self.speed, math.pi))  # yaw = pi

        return traj

    def publish_reference(self):
        #if self.step >= len(self.traj):
        #    return  # stop at end of path

        x, y, yaw = self.traj[self.step]

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.publisher_.publish(pose)

        # Advance through trajectory
        if self.forward:
            self.step += 1
            if self.step >= len(self.traj):
                self.forward = False
                self.step = len(self.traj) - 1
        else:
            self.step -= 1
            if self.step < 0:
                self.forward = True
                self.step = 0

def main(args=None):
    print("Reference node started")
    rclpy.init(args=args)
    reference_node = reference()
    rclpy.spin(reference_node)
    reference_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()