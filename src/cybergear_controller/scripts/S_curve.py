#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math

class SCurvePublisher(Node):
    def __init__(self):
        super().__init__('s_curve_publisher')
        # Create a publisher for Float32MultiArray messages on topic "s_curve"
        self.publisher_ = self.create_publisher(Float32MultiArray, 's_curve', 10)
        self.timer_period = 1.0/500.0  # seconds (10 ms)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.step = 0
        self.total_steps = 100  # Number of steps in the S-curve trajectory
        self.get_logger().info("S-Curve publisher node started.")

    def timer_callback(self):
        # Compute S-curve value using a logistic function:
        # value = 1 / (1 + exp(-k*(step - mid)))
        k = 0.2
        mid = self.total_steps / 2.0
        value = 1.0 / (1.0 + math.exp(-k * (self.step - mid)))
        
        # Create a Float32MultiArray message with 6 identical elements
        msg = Float32MultiArray()
        msg.data = [value] * 6
        
        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().debug(f'Published: {msg.data}')
        
        # Update the step; reset to zero if finished one cycle
        self.step = (self.step + 1) % (self.total_steps + 1)

def main(args=None):
    rclpy.init(args=args)
    node = SCurvePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
