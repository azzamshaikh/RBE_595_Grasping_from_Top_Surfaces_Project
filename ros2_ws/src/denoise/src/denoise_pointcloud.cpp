#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>


using namespace std::chrono_literals;

/* This class inherits from rclcpp::Node and represents a ROS2 node that subscribes to a PointCloud2 topic, 
 * processes the incoming point cloud by applying a PassThrough filter, and publishes the filtered (denoised) 
 * point cloud to another topic. */
class Denoise : public rclcpp::Node
{
    public:
        // Constructor: Initializes the node, sets up the subscriptions, and creates a transform listener
        Denoise(): Node("denoise")
        {
            // Subscribe to the /realsense/points topic to receive point cloud data
            pointcloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/realsense/points", 10,
                std::bind(&Denoise::denoise_callback, this, std::placeholders::_1));

            // Publisher for denoised (filtered) point cloud data
            denoised_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/denoised_pointcloud_data", 10);

            // Create a TF buffer and listener to manage transformations, e.g., to get data between frames
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }

    private:
        // Callback function that processes incoming point clouds
        void denoise_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg){
            // Convert the incoming ROS point cloud message to a PCL point cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*cloud_msg,*cloud);
            
            // Denoising and downsampling logic:
            // Create a PassThrough filter to filter the point cloud along the z-axis
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("z"); 
            pass.setFilterLimits(0.0, 1.0);

            // Apply the filter and store the result in a new point cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
            pass.filter(*cloud_filtered);

            // Convert the filtered (denoised) PCL point cloud back to a ROS message
            sensor_msgs::msg::PointCloud2 denoised_cloud_msg;
            pcl::toROSMsg(*cloud_filtered, denoised_cloud_msg);
            denoised_cloud_msg.header.frame_id = "world";
            denoised_cloud_msg.header.stamp = this->get_clock()->now();

            // Publish the denoised point cloud
            denoised_pointcloud_publisher_->publish(denoised_cloud_msg);
        }

        // Setup subscribers and publishers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr denoised_pointcloud_publisher_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char * argv[])
{
  // Initialize the ROS2 system
  rclcpp::init(argc, argv);
  
  // Create an instance of the Denoise node and start spinning to process callbacks
  rclcpp::spin(std::make_shared<Denoise>());
  
  // Shutdown the ROS2 system when the node is done
  rclcpp::shutdown();
  return 0;
}