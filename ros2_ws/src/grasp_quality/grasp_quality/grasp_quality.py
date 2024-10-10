import rclpy
import rclpy.duration
from rclpy.node import Node

import rclpy.node
import rclpy.timer
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
import ros2_numpy
import numpy as np
import time




class GraspQuality(Node):

    def __init__(self):
        super().__init__('grasp_quality')
        self.wait_for_topic('/cluster_0_grasp')
        self.wait_for_topic('/cluster_1_grasp')
        self.wait_for_topic('/cluster_2_grasp')
        self.wait_for_topic('/cluster_0_angle')
        self.wait_for_topic('/cluster_1_angle')
        self.wait_for_topic('/cluster_2_angle')
        self.cluster_0_rotations = None
        self.cluster_1_rotations = None
        self.cluster_2_rotations = None
        self.cluster_0_angle = self.create_subscription(
            Float32MultiArray,
            'cluster_0_angle',
            self.cluster_0_angle_callback,
            10)
        self.cluster_0_grasp = self.create_subscription(
            PointCloud2,
            'cluster_0_grasp',
            self.cluster_0_callback,
            10)
        self.cluster_1_angle = self.create_subscription(
            Float32MultiArray,
            'cluster_1_angle',
            self.cluster_1_angle_callback,
            10)
        self.cluster_1_grasp = self.create_subscription(
            PointCloud2,
            'cluster_1_grasp',
            self.cluster_1_callback,
            10)
        self.cluster_2_angle = self.create_subscription(
            Float32MultiArray,
            'cluster_2_angle',
            self.cluster_2_angle_callback,
            10)        
        self.cluster_2_grasp = self.create_subscription(
            PointCloud2,
            'cluster_2_grasp',
            self.cluster_2_callback,
            10)
        
    
    def wait_for_topic(self, topic_name):
        """
        Function to wait until the topic is available
        """
        self.get_logger().info(f"Waiting for topic {topic_name}...")
        while topic_name not in [name for name, _ in self.get_topic_names_and_types()]:
            time.sleep(0.5)
            self.get_logger().info(f"Still waiting for {topic_name}...")
        self.get_logger().info(f"{topic_name} is now available.")


    @staticmethod
    def w_x_r(c,o):
        """
        Function to compute the cross product
        """
        r = c-o
        return np.array([-r[1],r[0]])

    def compute_grasp_matrix(self, origin, contacts, rotation_matrix):
        """
        Computes the grasp matrix in three setps. 

        First step is to obtain the Pi matrix for the contact point. 
        Pi is in the form as follows:
            [I_3x3 S(ci-o)^T;
            0_3x3 I_3x3]

        where      
        :param ci: 3x1 contact vector
        :param o: 3x1 origin vector

        Second step is to obtain the Ri matrix for the contact point. The inverse is taken to define the N-frame in the Ci coordinate frame.
        Ri is in the form as follows:
            [ri 0_3x3;
            0_3x3 ri]
        where 
        :param ri: 3x3 rotation matrix that transforms the contact frame to the base frame. This form is before its inverse is taken.

        Third step is to compute the partial grasp matrix, Gi, for given contact point. The computation is 
        the matrix multiplication of Ri@Pi.

        After computing the partial grasp matrix, the final grasp matrix can be computed by concatenating the 
        partial grasp matrices into a single matrix. The concatentation results in the transpose of the grasp
        matrix, or G^T. To obtain the grasp matrix, G, the traspose is taken of G^T. 
        """
        G_i = []
        for idx in range(len(contacts)):
            Pi = np.vstack((np.hstack((np.eye(2), self.w_x_r(contacts[idx],origin).reshape((2,1)))),  np.hstack((np.zeros((1,2)), np.eye(1)))))
            Ri = rotation_matrix[idx]
            Gi = Ri@Pi
            Gi = np.array([[1,0,0],[0,1,0],[0,0,0]])@Gi
            Gi = np.linalg.inv(Ri)@Gi
            G_i.append(Gi)
        G_T = np.vstack(G_i)
        G = G_T.T
        return G
    
    @staticmethod
    def R(theta):
        """
        Function to output a rotation matrix
        """
        return np.round(np.array([[np.cos(theta),-np.sin(theta), 0],
                        [np.sin(theta), np.cos(theta), 0],
                        [0,0,1]]),3)
    
    @staticmethod
    def compute_grasp_metrics(G):
        """
        Function to extract the grasp metrics for a given grasp matrix
        """
        singular_values = np.linalg.svd(G, compute_uv=False)
        min_val = np.min(singular_values)
        max_val = np.max(singular_values)
        min_singular_value = min_val
        ellipsoid_metric = np.prod(singular_values)
        isotropy_index = min_val/max_val
        return min_singular_value, ellipsoid_metric, isotropy_index
        


    def cluster_0_callback(self,msg):
        """
        Call back for cluster 0
        """
        # Get the origins and contacts from the pointcloud
        origin, contacts = self.preprocess_pointcloud(msg)

        # COmpute the grasp matrix
        G = self.compute_grasp_matrix(origin, contacts, self.cluster_0_rotations)

        # COmpute the grasp metrics
        min_singular_value, ellipsoid_metric, isotropy_index = self.compute_grasp_metrics(G)

        # Print results
        print("For cluster 0\n\tSingular Value: {}\n\tEllipsoid: {}\n\tIsotropy Index: {}".format(min_singular_value,
                                                                                                  ellipsoid_metric,
                                                                                                  isotropy_index))


    def cluster_1_callback(self,msg):
        """
        Call back for cluster 1
        """
        # Get the origins and contacts from the pointcloud
        origin, contacts = self.preprocess_pointcloud(msg)

        # COmpute the grasp matrix
        G = self.compute_grasp_matrix(origin, contacts, self.cluster_1_rotations)

        # COmpute the grasp metrics
        min_singular_value, ellipsoid_metric, isotropy_index = self.compute_grasp_metrics(G)

        # Print results
        print("For cluster 1\n\tSingular Value: {}\n\tEllipsoid: {}\n\tIsotropy Index: {}".format(min_singular_value,
                                                                                                  ellipsoid_metric,
                                                                                                  isotropy_index))

    def cluster_2_callback(self,msg):
        """
        Call back for cluster 1
        """
        # Get the origins and contacts from the pointcloud
        origin, contacts = self.preprocess_pointcloud(msg)

        # COmpute the grasp matrix
        G = self.compute_grasp_matrix(origin, contacts, self.cluster_2_rotations)
        
        # COmpute the grasp metrics
        min_singular_value, ellipsoid_metric, isotropy_index = self.compute_grasp_metrics(G)
        
        # Print results
        print("For cluster 2\n\tSingular Value: {}\n\tEllipsoid: {}\n\tIsotropy Index: {}".format(min_singular_value,
                                                                                                  ellipsoid_metric,
                                                                                                  isotropy_index))
        

    def cluster_0_angle_callback(self, msg: Float32MultiArray):
        """
        Call back for cluster 0 angles
        """
        # Convert the orientations to rotation matrices. The order is GP1 and GP2. 
        rotations = []
        for angle in msg.data:
            rotations.append(self.R(angle))
        self.cluster_0_rotations = rotations

    def cluster_1_angle_callback(self, msg: Float32MultiArray):
        """
        Call back for cluster 1 angles
        """
        # Convert the orientations to rotation matrices. The order is GP1 and GP2. 
        rotations = []
        for angle in msg.data:
            rotations.append(self.R(angle))
        self.cluster_1_rotations = rotations

    def cluster_2_angle_callback(self, msg: Float32MultiArray):
        """
        Call back for cluster 2 angles
        """
        # Convert the orientations to rotation matrices. The order is GP1 and GP2. 
        rotations = []
        for angle in msg.data:
            rotations.append(self.R(angle))
        self.cluster_2_rotations = rotations
    
    @staticmethod
    def preprocess_pointcloud(msg):
        """
        Function to process the grasp point cloud. Needs data in com, gp1, gp2 format
        Ouputs the origin (com), and the two contacts (gp1, gp2)
        """
        xyz = ros2_numpy.point_cloud2.point_cloud2_to_array(msg)['xyz']
        origin = xyz[0,:]
        contacts = []
        for idx in range(1,xyz.shape[0]):
            contacts.append(xyz[idx,:])
        return origin, contacts



def main(args=None):
    rclpy.init(args=args)

    grasp_quality = GraspQuality()
    rclpy.node.get_logger("Main").info(f"Waiting for all other services to load")
    time.sleep(10)
    rclpy.node.get_logger("Main").info(f"All services loaded.")
    rclpy.spin(grasp_quality)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    grasp_quality.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()