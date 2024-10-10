#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>
#include <pcl/filters/extract_indices.h>

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MajorPlaneDetection : public rclcpp::Node
{
    public:
        // Constructor: Initializes the node, sets up subscriptions and publishers, and creates a transform listener
        MajorPlaneDetection(): Node("plane_detection")
        {
            pointcloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/denoised_pointcloud_data", 10,
                std::bind(&MajorPlaneDetection::plane_detection_callback, this, std::placeholders::_1));

            object_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/object_pointcloud_data", 10);

            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }

    private:
        // Callback function that processes the point cloud to detect and remove the major plane
        void plane_detection_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg){
            // Convert the incoming ROS point cloud message to a PCL point cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*cloud_msg,*cloud);
            
            // Implement plane detection using RANSAC (Random Sample Consensus)
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

            // Set up the segmentation object for plane detection
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setDistanceThreshold (0.01);
            seg.setInputCloud (cloud);
            seg.segment (*inliers, *coefficients);
            
            // If no plane was detected, log an error and return
            if (inliers->indices.size () == 0)
            {
                PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
                return;
            }
            // Optionally, print the coefficients and inliers (commented out for now)
            // std::cerr << "Model coefficients: " << coefficients->values[0] << " " << coefficients->values[1] << " "<< coefficients->values[2] << " " << coefficients->values[3] << std::endl;
            // std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
            
            // Create a new point cloud to store the points not belonging to the detected plane (i.e., the objects)
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ExtractIndices<pcl::PointXYZ> extract;

            // Set the input cloud and the indices of the plane inliers
    	    extract.setInputCloud(cloud);          // Set the input point cloud
    	    extract.setIndices(inliers);           // Set the indices of the points to be removed
    	    extract.setNegative(true);             // True to remove the inliers (instead of keeping them)
    	    extract.filter(*cloud_f);// Perform the filtering	

            // Convert the filtered point cloud (objects) back to a ROS message
            sensor_msgs::msg::PointCloud2 object_cloud_msg;
            pcl::toROSMsg(*cloud_f, object_cloud_msg); // make sure to change the reference input cloud that is being called here to the correct one that is output to ros
            object_cloud_msg.header.frame_id = "world";
            object_cloud_msg.header.stamp = this->get_clock()->now();

            // Publish the point cloud containing the objects
            object_pointcloud_publisher_->publish(object_cloud_msg);
        }
        
        // Setup subscribers and publishers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr object_pointcloud_publisher_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char * argv[])
{
  // Initialize the ROS2 system
  rclcpp::init(argc, argv);
    
  // Create an instance of the Denoise node and start spinning to process callbacks
  rclcpp::spin(std::make_shared<MajorPlaneDetection>());
  
  // Shutdown the ROS2 system when the node is done
  rclcpp::shutdown();
  return 0;
}