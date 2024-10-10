import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
import ros2_numpy
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from scipy.spatial import KDTree
import time
from std_msgs.msg import Float32MultiArray

class PlanarGrasp(Node):

    def __init__(self):
        super().__init__('planar_grasp')
        self.wait_for_topic('/cluster_0_convex')
        self.wait_for_topic('/cluster_1_convex')
        self.wait_for_topic('/cluster_2_convex')
        self.cluster0_edge_pc = np.empty((3,1))
        self.cluster1_edge_pc = np.empty((3,1))
        self.cluster2_edge_pc = np.empty((3,1))
        self.cluster0_kd_tree = None
        self.cluster1_kd_tree = None
        self.cluster2_kd_tree = None
        self.cluster0_convex_sub = self.create_subscription(
            PointCloud2,
            'cluster_0_convex',
            self.cluster0_convex_callback,
            10)
        self.cluster1_convex_sub = self.create_subscription(
            PointCloud2,
            'cluster_1_convex',
            self.cluster1_convex_callback,
            10)
        self.cluster2_convex_sub = self.create_subscription(
            PointCloud2,
            'cluster_2_convex',
            self.cluster2_convex_callback,
            10)
        self.wait_for_topic('/cluster_0_top')
        self.wait_for_topic('/cluster_1_top')
        self.wait_for_topic('/cluster_2_top')
        self.cluster0_top_sub = self.create_subscription(
            PointCloud2,
            'cluster_0_top',
            self.cluster0_top_callback,
            10)
        self.cluster1_top_sub = self.create_subscription(
            PointCloud2,
            'cluster_1_top',
            self.cluster1_top_callback,
            10)
        self.cluster2_top_sub = self.create_subscription(
            PointCloud2,
            'cluster_2_top',
            self.cluster2_top_callback,
            10)
        
        self.cluster0_grasp_pub_ = self.create_publisher(PointCloud2, 'cluster_0_grasp', 10)
        self.cluster0_angle_pub_ = self.create_publisher(Float32MultiArray, 'cluster_0_angle',10)
        self.cluster1_grasp_pub_ = self.create_publisher(PointCloud2, 'cluster_1_grasp', 10)
        self.cluster1_angle_pub_ = self.create_publisher(Float32MultiArray, 'cluster_1_angle',10)
        self.cluster2_grasp_pub_ = self.create_publisher(PointCloud2, 'cluster_2_grasp', 10)
        self.cluster2_angle_pub_ = self.create_publisher(Float32MultiArray, 'cluster_2_angle',10)
    
    def wait_for_topic(self, topic_name):
        self.get_logger().info(f"Waiting for topic {topic_name}...")
        while topic_name not in [name for name, _ in self.get_topic_names_and_types()]:
            time.sleep(0.5)
            self.get_logger().info(f"Still waiting for {topic_name}...")
        self.get_logger().info(f"{topic_name} is now available.")


    def cluster0_convex_callback(self, msg):
        """
        Cluster callback
        """
        # Extract the point cloud and create a np array and kd tree out of it
        pc_array = ros2_numpy.point_cloud2.point_cloud2_to_array(msg)
        xyz = pc_array['xyz']
        self.cluster0_edge_pc = xyz
        self.cluster0_kd_tree = KDTree(xyz)

    def cluster0_top_callback(self, msg):
        """
        Cluster grasp point callback
        """
        # Obtain the grasp points and orientation and then publish them
        cloud, angles = self.obtain_grasp_points(msg, self.cluster0_kd_tree, self.cluster0_edge_pc)
        self.cluster0_grasp_pub_.publish(cloud)
        self.cluster0_angle_pub_.publish(angles)
    

    def cluster1_convex_callback(self, msg):
        """
        Cluster callback
        """
        # Extract the point cloud and create a np array and kd tree out of it
        pc_array = ros2_numpy.point_cloud2.point_cloud2_to_array(msg)
        xyz = pc_array['xyz']
        self.cluster1_edge_pc = xyz
        self.cluster1_kd_tree = KDTree(xyz)

    def cluster1_top_callback(self, msg):
        """
        Cluster grasp point callback
        """
        # Obtain the grasp points and orientation and then publish them
        cloud, angles = self.obtain_grasp_points(msg, self.cluster1_kd_tree, self.cluster1_edge_pc)
        self.cluster1_grasp_pub_.publish(cloud)
        self.cluster1_angle_pub_.publish(angles)

    def cluster2_convex_callback(self, msg):
        """
        Cluster callback
        """
        # Extract the point cloud and create a np array and kd tree out of it
        pc_array = ros2_numpy.point_cloud2.point_cloud2_to_array(msg)
        xyz = pc_array['xyz']
        self.cluster2_edge_pc = xyz
        self.cluster2_kd_tree = KDTree(xyz)

    def cluster2_top_callback(self, msg):
        """
        Cluster grasp point callback
        """
        # Obtain the grasp points and orientation and then publish them
        cloud, angles = self.obtain_grasp_points(msg, self.cluster2_kd_tree, self.cluster2_edge_pc)
        self.cluster2_grasp_pub_.publish(cloud)
        self.cluster2_angle_pub_.publish(angles)

    @staticmethod
    def obtain_normal_and_theta(neighbors, point):
        """
        Get the grasp pointn normal and orientation
        """
        point = point[:-1]  # remove z
        neighbors = neighbors[:,:-1] # remove z

        # Compute the covariance matrix and perform PCA
        cov_matrix = np.cov(neighbors.T)
        eigvals, eigvecs = np.linalg.eig(cov_matrix)

        # The eigenvector corresponding to the smallest eigenvalue is the normal direction
        normal = eigvecs[:, np.argmin(eigvals)]
        
        # Normalize the normal vector
        normal = point / np.linalg.norm(point)
        
        # Flip normal
        normal = -normal

        # Get the orientation
        angle = np.arctan2(normal[1], normal[0],dtype=np.float32)

        return angle
    
    def obtain_grasp_points(self,msg, kd_tree : KDTree, edge_pc):
        """
        Get the grasp points for a give tree and point cloud 
        """
        # Extact the x y and z arrays
        pc_array = ros2_numpy.point_cloud2.point_cloud2_to_array(msg)
        xyz = pc_array['xyz']
        x = xyz[:,0]
        y = xyz[:,1]
        z = xyz[:,2]
        
        # Get the center of mass
        com_x = np.average(x)
        com_y = np.average(y)
        com_z = np.average(z)
        center_of_mass = np.array([com_x, com_y, com_z], dtype=np.float32)

        # Initialize the data structures for the return values
        data = np.row_stack((center_of_mass))
        angles = Float32MultiArray()
        angles.data = np.array([0,0], dtype=np.float32).tolist()
        
        if kd_tree is not None:
            # Get the closest point to the com
            dist, idx = kd_tree.query([com_x, com_y, com_z])

            # Convert that to an np array
            gp1 = np.array(edge_pc[idx], dtype=np.float32)

            # Get angle for gp1
            _, indicies = kd_tree.query(gp1,k=3)
            neighbors = edge_pc[indicies]
            gp1_angle = self.obtain_normal_and_theta(neighbors,gp1)

            # Create a vector between gp1 and com
            vector = gp1 - center_of_mass
            
            # Offset the vector to the opposite side
            guess = center_of_mass - vector
            
            # Get the closest point from the tree to the guess
            dist2, idx2 = kd_tree.query(guess)

            # Convert that to an np array
            gp2 = np.array(edge_pc[idx2], dtype=np.float32)

            # Get angle for gp2
            _, indicies = kd_tree.query(gp2,k=3)
            neighbors = edge_pc[indicies]
            gp2_angle = self.obtain_normal_and_theta(neighbors,gp2)

            # Stack the data for formatting the point cloud
            data = np.row_stack((center_of_mass, gp1, gp2))
            
            # Create the angles data structure
            angles.data = np.array( [gp1_angle, gp2_angle], dtype=np.float32).tolist()
        
        # Convert the data to a sensor pointcloud
        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = 'world'

        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        cloud.data = data.tobytes()
        cloud.width = data.shape[0]
        cloud.height = 1
        cloud.is_bigendian = False
        cloud.point_step = 12 # size of each point in bytes
        cloud.row_step = cloud.point_step*cloud.width
            
        return cloud, angles

def main(args=None):
    rclpy.init(args=args)

    planar_grasp = PlanarGrasp()

    rclpy.spin(planar_grasp)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    planar_grasp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()