import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
import ros2_numpy
import numpy as np
import numpy.typing as npt
import sensor_msgs_py.point_cloud2 as pc2
from scipy.spatial import KDTree
import time

class PlanarGrasp(Node):

    def __init__(self):
        super().__init__('planar_grasp')
        self.wait_for_topic('/planar_surface_0')
        self.wait_for_topic('/planar_surface_1')
        self.wait_for_topic('/planar_surface_2')
        self.cluster0_convex_sub = self.create_subscription(
            PointCloud2,
            'planar_surface_0',
            self.cluster0_grasp_callback,
            10)
        self.cluster1_convex_sub = self.create_subscription(
            PointCloud2,
            'planar_surface_1',
            self.cluster1_grasp_callback,
            10)
        self.cluster2_convex_sub = self.create_subscription(
            PointCloud2,
            'planar_surface_2',
            self.cluster2_grasp_callback,
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


    def cluster0_grasp_callback(self, msg):
        x, y = self.pointcloud_to_x_and_y(msg)

        # add implementation here

        #cloud = self.to_pointcloud_msg() # make sure to convert the data to a cloud
        self.cluster0_grasp_pub_.publish(msg) #update to publish the cloud 
    

    def cluster1_grasp_callback(self, msg):
        x, y = self.pointcloud_to_x_and_y(msg)

        # add implementation here

        #cloud = self.to_pointcloud_msg() # make sure to convert the data to a cloud
        self.cluster1_grasp_pub_.publish(msg)


    def cluster2_grasp_callback(self, msg):
        x, y = self.pointcloud_to_x_and_y(msg)
        
        # add implementation here

        #cloud = self.to_pointcloud_msg() # make sure to convert the data to a cloud
        self.cluster2_grasp_pub_.publish(msg)

    @staticmethod
    def pointcloud_to_x_and_y(msg):
        xyz = ros2_numpy.point_cloud2.point_cloud2_to_array(msg)['xyz']
        x = xyz[:,0]
        y = xyz[:,1]
        z = xyz[:,2]
        return x, y 
    
    def to_pointcloud_msg(self, data:npt.NDArray[np.float32]):
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