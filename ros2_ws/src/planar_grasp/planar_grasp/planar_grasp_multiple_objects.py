import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
import ros2_numpy
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from scipy.spatial import KDTree
import time

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
        self.cluster1_grasp_pub_ = self.create_publisher(PointCloud2, 'cluster_1_grasp', 10)
        self.cluster2_grasp_pub_ = self.create_publisher(PointCloud2, 'cluster_2_grasp', 10)
    
    def wait_for_topic(self, topic_name):
        self.get_logger().info(f"Waiting for topic {topic_name}...")
        while topic_name not in [name for name, _ in self.get_topic_names_and_types()]:
            time.sleep(0.5)
            self.get_logger().info(f"Still waiting for {topic_name}...")
        self.get_logger().info(f"{topic_name} is now available.")


    def cluster0_convex_callback(self, msg):
        pc_array = ros2_numpy.point_cloud2.point_cloud2_to_array(msg)
        xyz = pc_array['xyz']
        #self.edge_pointcloud = np.array([p[:3] for p in pc2.read_points(msg, skip_nans=True)])
        self.cluster0_edge_pc = xyz
        self.cluster0_kd_tree = KDTree(xyz)
        #self.get_logger().info(f"{self.edge_pointcloud}.")

    def cluster0_top_callback(self, msg):
        pc_array = ros2_numpy.point_cloud2.point_cloud2_to_array(msg)
        xyz = pc_array['xyz']
        x = xyz[:,0]
        y = xyz[:,1]
        z = xyz[:,2]
        com_x = np.average(x)
        com_y = np.average(y)
        com_z = np.average(z)

        center_of_mass = np.array([com_x, com_y, com_z], dtype=np.float32)
        # center_of_mass = np.array([[com_x, com_y, com_z]], dtype=np.float32)

        # cloud.data = center_of_mass.tobytes()
        # cloud.width = center_of_mass.shape[0]
        # cloud.height = 1
        # cloud.is_bigendian = False
        # cloud.point_step = 12 # size of each point in bytes
        # cloud.row_step = cloud.point_step*cloud.width

        data = np.row_stack((center_of_mass))

        #####
        if self.cluster0_kd_tree is not None:
            dist, idx = self.cluster0_kd_tree.query([com_x, com_y, com_z])

            gp1 = np.array(self.cluster0_edge_pc[idx], dtype=np.float32)
            # gp1 = np.array([self.cluster0_edge_pc[idx]], dtype=np.float32)
            # gp1_cloud = PointCloud2()
            # gp1_cloud.header.stamp = self.get_clock().now().to_msg()
            # gp1_cloud.header.frame_id = 'world'

            # gp1_cloud.fields = [
            #     PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            #     PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            #     PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            # ]

            # gp1_cloud.data = gp1.tobytes()
            # gp1_cloud.width = gp1.shape[0]
            # gp1_cloud.height = 1
            # gp1_cloud.is_bigendian = False
            # gp1_cloud.point_step = 12 # size of each point in bytes
            # gp1_cloud.row_step = gp1_cloud.point_step*gp1_cloud.width
            # self.gp1_publisher_.publish(gp1_cloud)

            vector = gp1 - center_of_mass
            guess = center_of_mass - vector
            
            dist2, idx2 = self.cluster0_kd_tree.query(guess)
            gp2 = np.array(self.cluster0_edge_pc[idx2], dtype=np.float32)
            # gp2 = np.array([self.edge_pointcloud[idx2]], dtype=np.float32)
            # gp2_cloud = PointCloud2()
            # gp2_cloud.header.stamp = self.get_clock().now().to_msg()
            # gp2_cloud.header.frame_id = 'world'

            # gp2_cloud.fields = [
            #     PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            #     PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            #     PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            # ]

            # gp2_cloud.data = gp2.tobytes()
            # gp2_cloud.width = gp2.shape[0]
            # gp2_cloud.height = 1
            # gp2_cloud.is_bigendian = False
            # gp2_cloud.point_step = 12
            # gp2_cloud.row_step = gp2_cloud.point_step*gp2_cloud.width
            # self.gp2_publisher_.publish(gp2_cloud)
            data = np.row_stack((center_of_mass, gp1, gp2))

        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = 'world'

        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        cloud.data = data.tobytes()
        cloud.width = center_of_mass.shape[0]
        cloud.height = 1
        cloud.is_bigendian = False
        cloud.point_step = 12 # size of each point in bytes
        cloud.row_step = cloud.point_step*cloud.width
            
        self.cluster0_grasp_pub_.publish(cloud)
    

    def cluster1_convex_callback(self, msg):
        pc_array = ros2_numpy.point_cloud2.point_cloud2_to_array(msg)
        xyz = pc_array['xyz']
        #self.edge_pointcloud = np.array([p[:3] for p in pc2.read_points(msg, skip_nans=True)])
        self.cluster1_edge_pc = xyz
        self.cluster1_kd_tree = KDTree(xyz)
        #self.get_logger().info(f"{self.edge_pointcloud}.")

    def cluster1_top_callback(self, msg):

        cloud = self.obtain_grasp_points(msg, self.cluster1_kd_tree, self.cluster1_edge_pc)
        self.cluster1_grasp_pub_.publish(cloud)

    def cluster2_convex_callback(self, msg):
        pc_array = ros2_numpy.point_cloud2.point_cloud2_to_array(msg)
        xyz = pc_array['xyz']
        #self.edge_pointcloud = np.array([p[:3] for p in pc2.read_points(msg, skip_nans=True)])
        self.cluster2_edge_pc = xyz
        self.cluster2_kd_tree = KDTree(xyz)
        #self.get_logger().info(f"{self.edge_pointcloud}.")

    def cluster2_top_callback(self, msg):

        cloud = self.obtain_grasp_points(msg, self.cluster2_kd_tree, self.cluster2_edge_pc)
        self.cluster2_grasp_pub_.publish(cloud)

    
    def obtain_grasp_points(self, msg, kd_tree : KDTree, edge_pc):
        pc_array = ros2_numpy.point_cloud2.point_cloud2_to_array(msg)
        xyz = pc_array['xyz']
        x = xyz[:,0]
        y = xyz[:,1]
        z = xyz[:,2]
        com_x = np.average(x)
        com_y = np.average(y)
        com_z = np.average(z)

        center_of_mass = np.array([com_x, com_y, com_z], dtype=np.float32)


        data = np.row_stack((center_of_mass))

        #####
        if kd_tree is not None:
            dist, idx = kd_tree.query([com_x, com_y, com_z])

            gp1 = np.array(edge_pc[idx], dtype=np.float32)

            vector = gp1 - center_of_mass
            guess = center_of_mass - vector
            
            dist2, idx2 = kd_tree.query(guess)
            gp2 = np.array(edge_pc[idx2], dtype=np.float32)
            data = np.row_stack((center_of_mass, gp1, gp2))

        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = 'world'

        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        cloud.data = data.tobytes()
        cloud.width = center_of_mass.shape[0]
        cloud.height = 1
        cloud.is_bigendian = False
        cloud.point_step = 12 # size of each point in bytes
        cloud.row_step = cloud.point_step*cloud.width
            
        return cloud






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