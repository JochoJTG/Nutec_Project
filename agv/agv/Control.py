#!/usr/bin/env python3.10
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32, Float64
import math

class Control(Node):
    def __init__(self):
        super().__init__('Control')

        self.desired_position_subscriber = self.create_subscription(Vector3, 'desired_position', self.desired_position_callback, 10)
        self.current_position_subscriber = self.create_subscription(Vector3, 'current_position', self.current_position_callback, 10)

        self.lf_wheel_vel_subscriber = self.create_subscription(Float64, 'lf_wheel', self.lf_wheel_callback, 10)
        self.rf_wheel_vel_subscriber = self.create_subscription(Float64, 'rf_wheel', self.rf_wheel_callback, 10)
        self.lr_wheel_vel_subscriber = self.create_subscription(Float64, 'lr_wheel', self.lr_wheel_callback, 10)
        self.rr_wheel_vel_subscriber = self.create_subscription(Float64, 'rr_wheel', self.rr_wheel_callback, 10)



        self.velocities_publisher = self.create_publisher(Vector3, 'velocities', 10)

        self.kp = 1
        self.kd = 1
        self.ki = 1

        self.desired_position = Vector3()
        self.current_position = Vector3()
        self.velocities = Vector3()

        self.timer = self.create_timer(0.1, self.velocity_control)
        self.time = 0.0

    def lf_wheel_callback(self, msg):
        # Callback function for left front wheel velocity
        self.lf_wheel = msg.data
        self.lf_wheel = self.lf_wheel % 2* math.pi  # Normalize to [0, 2*pi]
        self.lf_distance = self.lf_wheel / 2*math.pi*0.05  # Assuming wheel radius is 0.05m

    def rf_wheel_callback(self, msg):
        # Callback function for right front wheel velocity
        self.rf_wheel = msg.data
        self.rf_wheel = self.rf_wheel % 2* math.pi
        self.rf_distance = self.rf_wheel / 2*math.pi*0.05
    
    def lr_wheel_callback(self, msg):
        # Callback function for left rear wheel velocity
        self.lr_wheel = msg.data
        self.lr_wheel = self.lr_wheel % 2* math.pi
        self.lr_distance = self.lr_wheel / 2*math.pi*0.05
    
    def rr_wheel_callback(self, msg):
        # Callback function for right rear wheel velocity
        self.rr_wheel = msg.data
        self.rr_wheel = self.rr_wheel % 2* math.pi
        self.rr_distance = self.rr_wheel * 2*math.pi*0.05
        
    
    def desired_position_callback(self, msg):
        # Callback function for desired position
        self.desired_position = msg
    
    def current_position_callback(self, msg):
        # Callback function for current position
        self.current_position = msg 
    

    def velocity_control(self):

        print(self.lf_distance)
        print(self.rf_distance)
        print(self.lr_distance)
        print(self.rr_distance)


        # Control logic to calculate velocities
        error_x = self.desired_position.x - self.current_position.x
        error_y = self.desired_position.y - self.current_position.y
        #self.w_z = self.desired_position.z - self.current_position.z



        self.v_x = self.kp*error_x
        self.v_y = self.kp*error_y
        self.w_z = 0.0


        # Publish the velocities
        self.velocities.x = self.v_x
        self.velocities.y = self.v_y
        self.velocities.z = self.w_z

        self.velocities_publisher.publish(self.velocities)

        # Update time
        self.time += 0.1








def main(args=None):
    #Main function that initializes the GUI
    print("Control Node")
    rclpy.init(args=args)
    Control_node = Control()
    rclpy.spin(Control_node)
    Control_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"An error occurred: {e}")
        print("Exiting...")

