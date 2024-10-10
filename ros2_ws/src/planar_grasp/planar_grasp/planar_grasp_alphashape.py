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
import logging

# Set the logging level to ERROR or higher to suppress warnings
logging.getLogger().setLevel(logging.ERROR)


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
        """
        Function to wait until the topic is avialable
        """
        self.get_logger().info(f"Waiting for topic {topic_name}...")
        while topic_name not in [name for name, _ in self.get_topic_names_and_types()]:
            time.sleep(0.5)
            self.get_logger().info(f"Still waiting for {topic_name}...")
        self.get_logger().info(f"{topic_name} is now available.")


    def cluster0_grasp_callback(self, msg):
        """
        Callback for cluster0
        """
        # Convert the point cloud to x and y data
        x, y, z = self.pointcloud_to_x_and_y(msg)

        # Get the mean of the z data to represent the height
        z_mean = np.mean(z)
        
        # Convert the x and y data to an array
        points = np.array([x, y]).T

        # Compute the grasp points
        GP1, GP2, com = compute_grasp_points(points)

        # Create the vector orientation data
        angle = Float32MultiArray()
        angle.data = np.array([GP1[2], GP2[2]], dtype=np.float32).tolist()

        # Create the data structure for the grasp point cloud
        center_of_mass = np.array([com.x, com.y, z_mean], dtype=np.float32)
        grasp_point_1 = np.array([GP1[0], GP1[1], z_mean], dtype=np.float32)
        grasp_point_2 = np.array([GP2[0], GP2[1], z_mean], dtype=np.float32)
        data = np.row_stack((center_of_mass,grasp_point_1,grasp_point_2))

        # Convert the grasp data to a sensor msg
        cloud = self.to_pointcloud_msg(data) 

        # Publish both the angle and cloud
        self.cluster0_grasp_pub_.publish(cloud) 
        self.cluster0_angle_pub.publish(angle)


    def cluster1_grasp_callback(self, msg):
        """
        Callback for cluster1
        """
        # Convert the point cloud to x and y data
        x, y, z = self.pointcloud_to_x_and_y(msg)

        # Get the mean of the z data to represent the height
        z_mean = np.mean(z)
        
        # Convert the x and y data to an array
        points = np.array([x, y]).T

        # Compute the grasp points
        GP1, GP2, com = compute_grasp_points(points)

        # Create the vector orientation data
        angle = Float32MultiArray()
        angle.data = np.array([GP1[2], GP2[2]], dtype=np.float32).tolist()

        # Create the data structure for the grasp point cloud
        center_of_mass = np.array([com.x, com.y, z_mean], dtype=np.float32)
        grasp_point_1 = np.array([GP1[0], GP1[1], z_mean], dtype=np.float32)
        grasp_point_2 = np.array([GP2[0], GP2[1], z_mean], dtype=np.float32)
        data = np.row_stack((center_of_mass,grasp_point_1,grasp_point_2))

        # Convert the grasp data to a sensor msg
        cloud = self.to_pointcloud_msg(data) 

        # Publish both the angle and cloud
        self.cluster1_grasp_pub_.publish(cloud)
        self.cluster1_angle_pub.publish(angle)
        

    def cluster2_grasp_callback(self, msg):
        """
        Callback for cluster2
        """
        # Convert the point cloud to x and y data
        x, y, z = self.pointcloud_to_x_and_y(msg)

        # Get the mean of the z data to represent the height
        z_mean = np.mean(z)
        
        # Convert the x and y data to an array
        points = np.array([x, y]).T

        # Compute the grasp points
        GP1, GP2, com = compute_grasp_points(points)

        # Create the vector orientation data
        angle = Float32MultiArray()
        angle.data = np.array([GP1[2], GP2[2]], dtype=np.float32).tolist()

        # Create the data structure for the grasp point cloud
        center_of_mass = np.array([com.x, com.y, z_mean], dtype=np.float32)
        grasp_point_1 = np.array([GP1[0], GP1[1], z_mean], dtype=np.float32)
        grasp_point_2 = np.array([GP2[0], GP2[1], z_mean], dtype=np.float32)
        data = np.row_stack((center_of_mass,grasp_point_1,grasp_point_2))

        # Convert the grasp data to a sensor msg
        cloud = self.to_pointcloud_msg(data) 

        # Publish both the angle and cloud
        self.cluster2_grasp_pub_.publish(cloud)
        self.cluster2_angle_pub.publish(angle)

    @staticmethod
    def pointcloud_to_x_and_y(msg):
        """
        Function to extract the point cloud data to an array
        """
        xyz = ros2_numpy.point_cloud2.point_cloud2_to_array(msg)['xyz']
        x = xyz[:,0]
        y = xyz[:,1]
        z = xyz[:,2]
        return x, y, z
    
    def to_pointcloud_msg(self, data: npt.NDArray[np.float32]):
        """
        Function to convert an array to a sensor msg
        """
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
        cloud.point_step = 12  
        cloud.row_step = cloud.point_step * cloud.width

        return cloud


def get_normal_vector(segment, outward=True):
        """
        Function to get the nromal vector
        """
        # Extract the two points
        p1 = np.array(segment.coords[0])
        p2 = np.array(segment.coords[1])

        # Create a vector 
        direction_vector = p2 - p1

        # Create the normal vector and then normalize it
        normal_vector = np.array([-direction_vector[1], direction_vector[0]])
        normal_vector /= np.linalg.norm(normal_vector)

        # Ensure its outward
        if not outward:
            normal_vector = -normal_vector
        
        # Return the vector
        return normal_vector

def compute_grasp_points(points, alpha=0.01):
    """
    Function to get the grasp poitns
    """

    # Convert the x and y points to a alphashape polygon
    alpha_shape = alphashape.alphashape(points, alpha)

    if isinstance(alpha_shape, Polygon): # if it is not a polygon, something's wrong
        # Get the center of mass
        com = alpha_shape.centroid

        # Get grasp point 1
        closest_point = nearest_points(com, alpha_shape.exterior)[1]
        
        # Create a direction vector and get the closest point opposite of the center of mass,
        # That is grasp point 2
        direction_vector = np.array([closest_point.x - com.x, closest_point.y - com.y])
        opposite_point_guess = Point(com.x - 2 * direction_vector[0], com.y - 2 * direction_vector[1])
        line_to_opposite = LineString([com, opposite_point_guess])
        intersection = line_to_opposite.intersection(alpha_shape.exterior)

        # Extact grasp point 2
        if not intersection.is_empty:
            if intersection.geom_type == 'Point':
                opposite_point = intersection
            elif intersection.geom_type == 'MultiPoint':
                # If multiple points, take the outermost one (usually rare in polygons)
                opposite_point = max(intersection, key=lambda pt: np.linalg.norm(np.array([pt.x, pt.y]) - np.array([com.x, com.y])))

        # Get the line segments that are tangent to the point
        polygon_segments = list(alpha_shape.exterior.coords)
        closest_segment = min([(LineString([polygon_segments[i], polygon_segments[i + 1]]), i) for i in range(len(polygon_segments) - 1)],
            key=lambda seg: seg[0].distance(closest_point))[0]

        opposite_segment = min([(LineString([polygon_segments[i], polygon_segments[i + 1]]), i) for i in range(len(polygon_segments) - 1)],
            key=lambda seg: seg[0].distance(opposite_point))[0]

        # Get their normal
        closest_normal = get_normal_vector(closest_segment)
        opposite_normal = get_normal_vector(opposite_segment)
        
        # Compute their orientation
        closest_normal_orientation = np.arctan2(closest_normal[1], closest_normal[0])
        opposite_normal_orientation = np.arctan2(opposite_normal[1], opposite_normal[0])

        # Return
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