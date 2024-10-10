import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

class FindNormals(Node):
    """
    This node subscribes to the 'Boundary_Points' topic, which contains an array of (x, y) points
    representing the outer boundary of a 2D contour. It checks if the contour is closed, calculates
    the outward normal vectors at each point, and publishes an array of the input points along with 
    the angles (theta) of the normals with respect to the positive x-axis to the 
    'Boundary_Points_with_Normals' topic.
    """

    def __init__(self):
        super().__init__('find_normals')
        # Subscriber to listen for boundary points
        self.subscriber = self.create_subscription(
            Float32MultiArray,
            'Boundary_Points',
            self.boundary_points_callback,
            10)
        # Publisher to output boundary points with normals
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'Boundary_Points_with_Normals',
            10)
    
    def boundary_points_callback(self, msg):
        """
        Callback function for receiving boundary points.
        
        Args:
            msg (Float32MultiArray): The message containing an array of boundary points, reshaped to (N, 2).
        
        Returns:
            None: If the contour is open, an error is logged, and the function exits early.
        """
        points = np.array(msg.data).reshape(-1, 2)  # Reshape input data to (N, 2)
        if not self.is_closed(points):
            self.get_logger().error('Open contour detected!')  # Log error for open contour
            return

        normals = self.calculate_normals(points)  # Calculate normals for the boundary points
        normals_with_theta = self.append_theta(points, normals)  # Append theta values to points
        self.publish_normals(normals_with_theta)  # Publish the result

    def is_closed(self, points):
        """
        Check if the contour is closed by comparing the first and last points.
        
        Args:
            points (np.ndarray): Array of shape (N, 2) containing boundary points.
        
        Returns:
            bool: True if the contour is closed, otherwise False.
        """
        return np.allclose(points[0], points[-1])

    def calculate_normals(self, points):
        """
        Calculate the normal vectors for each edge of the contour.
        
        Args:
            points (np.ndarray): Array of shape (N, 2) containing boundary points.
        
        Returns:
            np.ndarray: Array of shape (N, 2) containing normalized normal vectors.
        """
        normals = []
        for i in range(len(points)):
            p1 = points[i]
            p2 = points[(i + 1) % len(points)]  # Wrap around to form a closed loop
            edge = p2 - p1  # Vector representation of the edge
            normal = np.array([-edge[1], edge[0]])  # Rotate 90 degrees to get normal
            normal /= np.linalg.norm(normal)  # Normalize the normal vector
            normals.append(normal)
        return np.array(normals)

    def append_theta(self, points, normals):
        """
        Append the angle (theta) of each normal vector with respect to the positive x-axis
        to the corresponding point.
        
        Args:
            points (np.ndarray): Array of shape (N, 2) containing boundary points.
            normals (np.ndarray): Array of shape (N, 2) containing normal vectors.
        
        Returns:
            np.ndarray: Flattened array of shape (N * 3,) containing (x, y, theta) for each point.
        """
        normals_with_theta = []
        for point, normal in zip(points, normals):
            theta = np.arctan2(normal[1], normal[0])  # Calculate angle in radians
            normals_with_theta.append([point[0], point[1], theta])  # Append point and theta
        return np.array(normals_with_theta).flatten()  # Flatten the array for publishing

    def publish_normals(self, normals_with_theta):
        """
        Publish the array of points with their corresponding normal angles.
        
        Args:
            normals_with_theta (np.ndarray): Flattened array of shape (N * 3,) to be published.
        
        Returns:
            None
        """
        msg = Float32MultiArray()
        msg.data = normals_with_theta.tolist()  # Convert to list for message
        self.publisher.publish(msg)  # Publish the message
        self.get_logger().info('Published normals with theta.')  # Log the publication

def main(args=None):
    rclpy.init(args=args)
    node = FindNormals()  # Create node instance
    rclpy.spin(node)  # Keep the node active
    node.destroy_node()  # Clean up the node
    rclpy.shutdown()  # Shutdown ROS

if __name__ == '__main__':
    main()