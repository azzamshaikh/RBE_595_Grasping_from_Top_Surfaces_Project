import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32MultiArray
import ros2_numpy
import numpy as np
import numpy.typing as npt
import sensor_msgs_py.point_cloud2 as pc2
from scipy.spatial import KDTree
import time
import alphashape
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import nearest_points

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
        self.cluster0_angle_pub = self.create_publisher(Float32MultiArray, 'cluster_0_angle', 10)
        self.cluster1_angle_pub = self.create_publisher(Float32MultiArray, 'cluster_1_angle', 10)
        self.cluster2_angle_pub = self.create_publisher(Float32MultiArray, 'cluster_2_angle', 10)
    
    def wait_for_topic(self, topic_name):
        self.get_logger().info(f"Waiting for topic {topic_name}...")
        while topic_name not in [name for name, _ in self.get_topic_names_and_types()]:
            time.sleep(0.5)
            self.get_logger().info(f"Still waiting for {topic_name}...")
        self.get_logger().info(f"{topic_name} is now available.")


    def cluster0_grasp_callback(self, msg):
        x, y = self.pointcloud_to_x_and_y(msg)

        # add implementation here
        points = np.array([x, y]).T
        GP1, GP2, com = compute_grasp_points(points)

        normals = np.array([GP1[2], GP2[2]])
        #convert to numpy float32
        GP1 = np.array(GP1, dtype=np.float32)
        GP2 = np.array(GP2, dtype=np.float32)
        
        cloud_GP1 = self.to_pointcloud_msg(GP1)
        cloud_GP2 = self.to_pointcloud_msg(GP2)
        cloud_com = self.to_pointcloud_msg(np.array([com], dtype=np.float32))

        #cloud = self.to_pointcloud_msg() # make sure to convert the data to a cloud
        self.cluster0_grasp_pub_.publish(msg) #update to publish the cloud 

        #publish the angle
        angle = Float32MultiArray()
        angle.data = normals
        angle.layout.data_offset = 2
        self.cluster0_angle_pub.publish(angle)


    def cluster1_grasp_callback(self, msg):
        x, y = self.pointcloud_to_x_and_y(msg)

        # add implementation here
        points = np.array([x, y]).T
        GP1, GP2, com = compute_grasp_points(points)
        
        cloud_GP1 = self.to_pointcloud_msg(GP1)
        cloud_GP2 = self.to_pointcloud_msg(GP2)
        cloud_com = self.to_pointcloud_msg(np.array([com], dtype=np.float32))

        normals = np.array([GP1[2], GP2[2]])

        #cloud = self.to_pointcloud_msg() # make sure to convert the data to a cloud
        self.cluster1_grasp_pub_.publish(msg)

        angle = Float32MultiArray()
        angle.data = normals
        angle.layout.data_offset = 2
        self.cluster1_angle_pub.publish(angle)


    def cluster2_grasp_callback(self, msg):
        x, y = self.pointcloud_to_x_and_y(msg)
        
        # add implementation here
        points = np.array([x, y]).T
        GP1, GP2, com = compute_grasp_points(points)
        
        cloud_GP1 = self.to_pointcloud_msg(GP1)
        cloud_GP2 = self.to_pointcloud_msg(GP2)
        cloud_com = self.to_pointcloud_msg(np.array([com], dtype=np.float32))

        normals = np.array([GP1[2], GP2[2]])

        #cloud = self.to_pointcloud_msg() # make sure to convert the data to a cloud
        self.cluster2_grasp_pub_.publish(msg)

        angle = Float32MultiArray()
        angle.data = normals
        angle.layout.data_offset = 2
        self.cluster2_angle_pub.publish(angle)

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


def get_normal_vector(segment, outward=True):

        p1 = np.array(segment.coords[0])
        p2 = np.array(segment.coords[1])

        direction_vector = p2 - p1

        normal_vector = np.array([-direction_vector[1], direction_vector[0]])
        normal_vector /= np.linalg.norm(normal_vector)

        if not outward:
            normal_vector = -normal_vector
        
        return normal_vector

def compute_grasp_points(points, alpha=0.01):

    alpha_shape = alphashape.alphashape(points, alpha)

    if isinstance(alpha_shape, Polygon): # if it is not a polygon, something's wrong
        com = alpha_shape.centroid
        closest_point = nearest_points(com, alpha_shape.exterior)[1]
        
        direction_vector = np.array([closest_point.x - com.x, closest_point.y - com.y])
        opposite_point_guess = Point(com.x - 2 * direction_vector[0], com.y - 2 * direction_vector[1])
        line_to_opposite = LineString([com, opposite_point_guess])
        intersection = line_to_opposite.intersection(alpha_shape.exterior)

        if not intersection.is_empty:
            if intersection.geom_type == 'Point':
                opposite_point = intersection
            elif intersection.geom_type == 'MultiPoint':
                # If multiple points, take the outermost one (usually rare in polygons)
                opposite_point = max(intersection, key=lambda pt: np.linalg.norm(np.array([pt.x, pt.y]) - np.array([com.x, com.y])))

        polygon_segments = list(alpha_shape.exterior.coords)
        closest_segment = min([(LineString([polygon_segments[i], polygon_segments[i + 1]]), i) for i in range(len(polygon_segments) - 1)],
            key=lambda seg: seg[0].distance(closest_point))[0]

        opposite_segment = min([(LineString([polygon_segments[i], polygon_segments[i + 1]]), i) for i in range(len(polygon_segments) - 1)],
            key=lambda seg: seg[0].distance(opposite_point))[0]

        closest_normal = get_normal_vector(closest_segment)
        opposite_normal = get_normal_vector(opposite_segment)
        
        closest_normal_orientation = np.arctan2(closest_normal[1], closest_normal[0])
        opposite_normal_orientation = np.arctan2(opposite_normal[1], opposite_normal[0])

        return [closest_point.x, closest_point.y, closest_normal_orientation], [opposite_point.x, opposite_point.y, opposite_normal_orientation], com

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