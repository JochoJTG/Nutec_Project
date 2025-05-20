#!/usr/bin/env python3.10

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import  Vector3
from std_msgs.msg import Float32
import math

class reference(Node):
    def __init__(self):
        super().__init__('reference')



        self.timer = self.create_timer(0.1, self.timer_callback)
        self.time = 0.0

    def timer_callback(self):

        

        print("Publishing")

def main(args=None):
    print("Reference node started")
    rclpy.init(args=args)
    reference_node = reference()
    rclpy.spin(reference_node)
    reference_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()