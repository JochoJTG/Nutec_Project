#!/usr/bin/env python3.10
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
import math

class Kinematics(Node):
    def __init__(self):
        super().__init__('kinematics')
        self.publisher_ = self.create_publisher(Vector3, 'position', 10)
        self.publisher2_ = self.create_publisher(Float32, 'angle', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.time = 0.0
        self.position = Vector3()
        self.angle = Float32()
        self.position.x = 0.0