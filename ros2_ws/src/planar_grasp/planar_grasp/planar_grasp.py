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
        self.wait_for_topic('/convex_hull_pointcloud_data')
        self.edge_pointcloud = np.empty((3,1))
        self.kd_tree = None
        self.outer_edges_subscriber_ = self.create_subscription(
            PointCloud2,
            'convex_hull_pointcloud_data',
            self.outer_edges_callback,
            10)
        self.wait_for_topic('/top_of_object')
        self.subscription = self.create_subscription(
            PointCloud2,
            'top_of_object',
            self.grasp_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.com_publisher_ = self.create_publisher(PointCloud2, 'center_of_mass', 10)
        self.gp1_publisher_ = self.create_publisher(PointCloud2, 'grasp_point_1', 10)
        self.gp2_publisher_ = self.create_publisher(PointCloud2, 'grasp_point_2', 10)
    
    def wait_for_topic(self, topic_name):
        self.get_logger().info(f"Waiting for topic {topic_name}...")
        while topic_name not in [name for name, _ in self.get_topic_names_and_types()]:
            time.sleep(0.5)
            self.get_logger().info(f"Still waiting for {topic_name}...")
        self.get_logger().info(f"{topic_name} is now available.")


    def outer_edges_callback(self, msg):
        pc_array = ros2_numpy.point_cloud2.point_cloud2_to_array(msg)
        xyz = pc_array['xyz']
        #self.edge_pointcloud = np.array([p[:3] for p in pc2.read_points(msg, skip_nans=True)])
        self.edge_pointcloud = xyz
        self.kd_tree = KDTree(xyz)
        #self.get_logger().info(f"{self.edge_pointcloud}.")

    def grasp_callback(self, msg):
        pc_array = ros2_numpy.point_cloud2.point_cloud2_to_array(msg)
        xyz = pc_array['xyz']
        x = xyz[:,0]
        y = xyz[:,1]
        z = xyz[:,2]
        com_x = np.average(x)
        com_y = np.average(y)
        com_z = np.average(z)

        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = 'world'

        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        center_of_mass = np.array([[com_x, com_y, com_z]], dtype=np.float32)

        cloud.data = center_of_mass.tobytes()
        cloud.width = center_of_mass.shape[0]
        cloud.height = 1
        cloud.is_bigendian = False
        cloud.point_step = 12 # size of each point in bytes
        cloud.row_step = cloud.point_step*cloud.width

        #####
        if self.kd_tree is not None:
            dist, idx = self.kd_tree.query([com_x, com_y, com_z])
            
            gp1 = np.array([self.edge_pointcloud[idx]], dtype=np.float32)
            gp1_cloud = PointCloud2()
            gp1_cloud.header.stamp = self.get_clock().now().to_msg()
            gp1_cloud.header.frame_id = 'world'

            gp1_cloud.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            ]

            gp1_cloud.data = gp1.tobytes()
            gp1_cloud.width = gp1.shape[0]
            gp1_cloud.height = 1
            gp1_cloud.is_bigendian = False
            gp1_cloud.point_step = 12 # size of each point in bytes
            gp1_cloud.row_step = gp1_cloud.point_step*gp1_cloud.width
            self.gp1_publisher_.publish(gp1_cloud)

            vector = gp1 - center_of_mass
            guess = center_of_mass - vector

            dist2, idx2 = self.kd_tree.query(guess[0])
            gp2 = np.array([self.edge_pointcloud[idx2]], dtype=np.float32)
            gp2_cloud = PointCloud2()
            gp2_cloud.header.stamp = self.get_clock().now().to_msg()
            gp2_cloud.header.frame_id = 'world'

            gp2_cloud.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            ]

            gp2_cloud.data = gp2.tobytes()
            gp2_cloud.width = gp2.shape[0]
            gp2_cloud.height = 1
            gp2_cloud.is_bigendian = False
            gp2_cloud.point_step = 12
            gp2_cloud.row_step = gp2_cloud.point_step*gp2_cloud.width
            self.gp2_publisher_.publish(gp2_cloud)




        self.com_publisher_.publish(cloud)






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