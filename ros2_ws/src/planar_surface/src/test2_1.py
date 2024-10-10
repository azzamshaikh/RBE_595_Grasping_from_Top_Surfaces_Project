#!/usr/bin/python3
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String # change String

class EvaluateGraspPoints(Node):
    
    def __init__(self):
        super().__init__('evaluate_grasp_points')
        self.subscriber = self.create_subscription(String,msg,self.timer_callback,10)
        self.publisher_ = self.create_publisher(String, 'topic', 10) #change String


    def timer_callback(self):
        msg = String()
        # msg.data = 
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def calculate_angle(self, v1, v2):
        """

        """
        # Calculate the dot product and the magnitudes of the vectors
        dot_product = np.dot(v1, v2)
        magnitude_v1 = np.linalg.norm(v1)
        magnitude_v2 = np.linalg.norm(v2)
        
        # Calculate the angle in radians
        angle = np.arccos(dot_product / (magnitude_v1 * magnitude_v2))
        print("angle: ",np.degrees(angle),"deg.")
        return np.degrees(angle)  # Convert to degrees if needed

    def vector_angle_between_points(self, p1, p2):
        """

        """
        # Extract the points and angles
        x1, y1, theta1 = p1
        x2, y2, theta2 = p2

        # Convert angles from degrees to radians for calculations
        theta1_rad = np.radians(theta1)
        theta2_rad = np.radians(theta2)

        # Create the direction vectors
        v1 = np.array([np.cos(theta1_rad), np.sin(theta1_rad)])
        v2 = np.array([np.cos(theta2_rad), np.sin(theta2_rad)])
        
        # Create the line vector from p1 to p2
        line_vector = np.array([x2 - x1, y2 - y1])
        
        # Calculate the angles between the vectors and the line vector
        angle1 = calculate_angle(v1, line_vector)
        angle2 = calculate_angle(v2, line_vector)

        return angle1, angle2



def main(args=None):
    # Example usage:
    p1 = (0, 0, 0)  # (x1, y1, theta1)
    p2 = (10, 10, 180)  # (x2, y2, theta2)

    



    angles = vector_angle_between_points(p1, p2)
    print("Angles between the vectors and the intersecting line:", angles)

    rclpy.init(args=args)
    evaluate_grasp_points = EvaluateGraspPoints()
    rclpy.spin(evaluate_grasp_points)
    evaluate_grasp_points.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()