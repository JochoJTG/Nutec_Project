#!/usr/bin/env python3.10
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
import math

class Kinematics(Node):
    def __init__(self):
        super().__init__('kinematics')

        self.wheel_vel_lf_publisher = self.create_publisher(Float64, 'wheel_vel/left_front', 10)
        self.wheel_vel_rf_publisher = self.create_publisher(Float64, 'wheel_vel/right_front', 10)
        self.wheel_vel_lr_publisher = self.create_publisher(Float64, 'wheel_vel/left_rear', 10)
        self.wheel_vel_rr_publisher = self.create_publisher(Float64, 'wheel_vel/right_rear', 10)


        self.wheel_vel_lf = Float64()
        self.wheel_vel_rf = Float64()
        self.wheel_vel_lr = Float64()
        self.wheel_vel_rr = Float64()

        self.timer = self.create_timer(0.5, self.vel)
        self.time = 0.0
        # self.publisher_ = self.create_publisher(Vector3, 'position', 10)
        # self.publisher2_ = self.create_publisher(Float32, 'angle', 10)
        # self.timer = self.create_timer(0.1, self.timer_callback)
        # self.time = 0.0
        # self.position = Vector3()
        # self.angle = Float32()
        # self.position.x = 0.0

        self.r, self.L, self.W = 0.05, 0.557, 0.694

        self.v_x = 0.0
        self.v_y = 0.0
        self.w_z = 0.0


    def vel(self):

        self.v_x = 0.1
        self.v_y = 0.0  

        front_left = (1/self.r) * (self.v_x - self.v_y - self.w_z*(self.L+self.W))
        front_right = (1/self.r) * (self.v_x + self.v_y + self.w_z*(self.L+self.W))
        left_rear = (1/self.r) * (self.v_x + self.v_y - self.w_z*(self.L+self.W))
        right_rear = (1/self.r) * (self.v_x - self.v_y + self.w_z*(self.L+self.W))


        self.wheel_vel_lf.data = front_left
        self.wheel_vel_rf.data = front_right
        self.wheel_vel_lr.data = left_rear
        self.wheel_vel_rr.data = right_rear


        self.wheel_vel_lf_publisher.publish(self.wheel_vel_lf)
        self.wheel_vel_rf_publisher.publish(self.wheel_vel_rf)
        self.wheel_vel_lr_publisher.publish(self.wheel_vel_lr)
        self.wheel_vel_rr_publisher.publish(self.wheel_vel_rr)

        self.time += 0.5





def main(args=None):
    #Main function that initializes the GUI
    print("Kineamatics Node")
    rclpy.init(args=args)
    Kinematics_node = Kinematics()
    rclpy.spin(Kinematics_node)
    Kinematics_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"An error occurred: {e}")
        print("Exiting...")

