#!/usr/bin/env python3.10
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
import math

class Kinematics(Node):
    