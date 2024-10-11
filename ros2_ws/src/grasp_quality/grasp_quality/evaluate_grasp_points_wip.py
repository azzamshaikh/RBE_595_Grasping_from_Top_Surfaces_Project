import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

class EvaluateGraspPoints(Node):
    """
    This node subscribes to the 'Boundary_Points_with_Normals' topic, which contains an array of 
    points along with their normal angles. It flips the normals to point inward, evaluates pairs of 
    points by calculating scores based on the angles between the normals and the line connecting the 
    points, and publishes the sorted results to the 'Evaluated_Grasp_Points' topic.
    """

    def __init__(self):
        super().__init__('evaluate_grasp_points')
        # Subscriber to listen for boundary points with normals
        self.subscriber = self.create_subscription(
            Float32MultiArray,
            'Boundary_Points_with_Normals',
            self.boundary_points_with_normals_callback,
            10)
        # Publisher to output evaluated grasp points
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'Evaluated_Grasp_Points',
            10)

    def boundary_points_with_normals_callback(self, msg):
        """
        Callback function for receiving boundary points with normals.
        
        Args:
            msg (Float32MultiArray): The message containing an array of points with their normals,
            reshaped to (N, 3).
        
        Returns:
            None: The function processes the data and evaluates pairs.
        """
        data = np.array(msg.data).reshape(-1, 3)  # Reshape input data to (N, 3)
        points = data[:, :2]  # Extract (x, y) points
        normals = data[:, 2]  # Extract normal angles

        # Flip normals to point inward by adding 180 degrees (π radians)
        flipped_normals = normals + np.pi  
        self.evaluate_pairs(points, flipped_normals)  # Evaluate pairs with flipped normals

    def evaluate_pairs(self, points, normals):
        """
        Evaluate pairs of points by calculating scores based on their normals.
        
        Args:
            points (np.ndarray): Array of shape (N, 2) containing (x, y) points.
            normals (np.ndarray): Array of shape (N,) containing flipped normal angles.
        
        Returns:
            None: The function publishes the sorted scores after evaluation.
        """
        scores = []
        for i in range(len(points)):
            for j in range(i + 1, len(points)):  # Ensure unique pairs (i, j) and not (j, i)
                score = self.calculate_score(points[i], points[j], normals[i], normals[j])
                scores.append((i, j, score))  # Append pair index and score
        
        # Sort pairs by score (lower scores are better)
        scores.sort(key=lambda x: x[2])
        self.publish_scores(scores)  # Publish the sorted scores

    def calculate_score(self, p1, p2, n1, n2):
        """
        Calculate the score for a pair of points based on the angles of their normals
        and the line connecting the two points. The score is the sum of these angles.
        
        Args:
            p1 (np.ndarray): The first point as a 1D array with (x, y) coordinates.
            p2 (np.ndarray): The second point as a 1D array with (x, y) coordinates.
            n1 (float): The normal angle for the first point in radians.
            n2 (float): The normal angle for the second point in radians.
        
        Returns:
            float: The score, which is the sum of the angle differences.
        """
        line_vector = p2 - p1  # Vector from p1 to p2
        line_angle = np.arctan2(line_vector[1], line_vector[0])  # Angle of the connecting line
        angle1 = abs(line_angle - n1)  # Angle difference for the first normal
        angle2 = abs(line_angle - n2)  # Angle difference for the second normal
        return angle1 + angle2  # Return the sum of the angles as the score

    def publish_scores(self, scores):
        """
        Publishes grasp point data to a topic.

        Args:
            self: (Instance of the class) Reference to the class instance.
            scores: (List[Tuple[int, int, float]]) A list of tuples containing grasp point data.
                - Each tuple represents (index_i, index_j, score).
                - index_i and index_j are integer indices.
                - score is a floating-point value representing the grasp quality score.

        Returns:
            None
        """
        
        msg = Float32MultiArray()
        for i, j, score in scores:
            msg.data.append(i)
            msg.data.append(j)
            msg.data.append(score)
        self.publisher.publish(msg)
        self.get_logger().info('Published evaluated grasp points.')

def main(args=None):
    rclpy.init(args=args)
    node = EvaluateGraspPoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()