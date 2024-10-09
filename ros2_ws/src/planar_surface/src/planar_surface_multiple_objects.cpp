#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <std_msgs/msg/float64_multi_array.hpp>
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
#include <rclcpp/wait_for_message.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include "clustering_itf/msg/string_array.hpp"

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class PlanarSurfaceMultipleObjects : public rclcpp::Node
{
    public:
        PlanarSurfaceMultipleObjects(): Node("planar_surface_multiple_objects")
        {

            cluster0_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/cluster_0", 10,
                std::bind(&PlanarSurfaceMultipleObjects::cluster0_callback, this, std::placeholders::_1));
            cluster1_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/cluster_1", 10,
                std::bind(&PlanarSurfaceMultipleObjects::cluster1_callback, this, std::placeholders::_1));
            cluster2_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/cluster_2", 10,
                std::bind(&PlanarSurfaceMultipleObjects::cluster2_callback, this, std::placeholders::_1));
            
            // Setup publishers for planar surfaces and float64 arrays
            cluster0_surface_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/planar_surface_0", 10);
            cluster1_surface_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/planar_surface_1", 10);
            cluster2_surface_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/planar_surface_2", 10);
            planar_surface_array_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/planar_surface_array", 10);

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster0_surface_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster1_surface_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster2_surface_pub;


            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }


    private:

        // Callback for each "cluster_" topic
        void cluster0_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            // Process the received point cloud data
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*msg,*cloud);

            // implement plane detection here
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setDistanceThreshold (0.001);
            seg.setInputCloud (cloud);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0)
            {
                PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
                return;
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr top_of_object(new pcl::PointCloud<pcl::PointXYZ>);
            for (size_t i = 0; i < inliers->indices.size (); ++i)
                top_of_object->push_back(cloud->points[inliers->indices[i]]);
            
            // Step 2: Filter points 3 mm below the top plane

            // Extract plane coefficients (a, b, c, d) from the detected plane
            float a = coefficients->values[0];
            float b = coefficients->values[1];
            float c = coefficients->values[2];
            float d = coefficients->values[3];

            // Calculate the offset for the new plane (3 mm below)
            float offset = 0.003 * sqrt(a * a + b * b + c * c);
            float d_new = d - offset;

            // Create a new point cloud for the filtered points
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

            // Iterate through the original point cloud and filter points below the new plane
            for (const auto& point : cloud->points)
            {
                float plane_distance = a * point.x + b * point.y + c * point.z + d_new;

                if (plane_distance <= 0) // Points below or on the new plane
                {
                    filtered_cloud->points.push_back(point);
                }
            }

            sensor_msgs::msg::PointCloud2 top_of_object_msg;
            pcl::toROSMsg(*filtered_cloud, top_of_object_msg); // make sure to change the reference input cloud that is being called here to the correct one that is output to ros
            top_of_object_msg.header.frame_id = "world";
            top_of_object_msg.header.stamp = this->get_clock()->now();
            cluster0_surface_pub->publish(top_of_object_msg);

        }

        // Callback for each "cluster_" topic
        void cluster1_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            // Process the received point cloud data
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*msg,*cloud);

            // implement plane detection here
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setDistanceThreshold (0.001);
            seg.setInputCloud (cloud);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0)
            {
                PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
                return;
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr top_of_object(new pcl::PointCloud<pcl::PointXYZ>);
            for (size_t i = 0; i < inliers->indices.size (); ++i)
                top_of_object->push_back(cloud->points[inliers->indices[i]]);
            
            // Step 2: Filter points 3 mm below the top plane

            // Extract plane coefficients (a, b, c, d) from the detected plane
            float a = coefficients->values[0];
            float b = coefficients->values[1];
            float c = coefficients->values[2];
            float d = coefficients->values[3];

            // Calculate the offset for the new plane (3 mm below)
            float offset = 0.003 * sqrt(a * a + b * b + c * c);
            float d_new = d - offset;

            // Create a new point cloud for the filtered points
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

            // Iterate through the original point cloud and filter points below the new plane
            for (const auto& point : cloud->points)
            {
                float plane_distance = a * point.x + b * point.y + c * point.z + d_new;

                if (plane_distance <= 0) // Points below or on the new plane
                {
                    filtered_cloud->points.push_back(point);
                }
            }

            sensor_msgs::msg::PointCloud2 top_of_object_msg;
            pcl::toROSMsg(*filtered_cloud, top_of_object_msg); // make sure to change the reference input cloud that is being called here to the correct one that is output to ros
            top_of_object_msg.header.frame_id = "world";
            top_of_object_msg.header.stamp = this->get_clock()->now();
            cluster1_surface_pub->publish(top_of_object_msg);

        }
        
        // Callback for each "cluster_" topic
        void cluster2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            // Process the received point cloud data
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*msg,*cloud);

            // implement plane detection here
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setDistanceThreshold (0.001);
            seg.setInputCloud (cloud);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0)
            {
                PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
                return;
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr top_of_object(new pcl::PointCloud<pcl::PointXYZ>);
            for (size_t i = 0; i < inliers->indices.size (); ++i)
                top_of_object->push_back(cloud->points[inliers->indices[i]]);
            
            // Step 2: Filter points 3 mm below the top plane

            // Extract plane coefficients (a, b, c, d) from the detected plane
            float a = coefficients->values[0];
            float b = coefficients->values[1];
            float c = coefficients->values[2];
            float d = coefficients->values[3];

            // Calculate the offset for the new plane (3 mm below)
            float offset = 0.003 * sqrt(a * a + b * b + c * c);
            float d_new = d - offset;

            // Create a new point cloud for the filtered points
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

            // Iterate through the original point cloud and filter points below the new plane
            for (const auto& point : cloud->points)
            {
                float plane_distance = a * point.x + b * point.y + c * point.z + d_new;

                if (plane_distance <= 0) // Points below or on the new plane
                {
                    filtered_cloud->points.push_back(point);
                }
            }

            sensor_msgs::msg::PointCloud2 top_of_object_msg;
            pcl::toROSMsg(*filtered_cloud, top_of_object_msg); // make sure to change the reference input cloud that is being called here to the correct one that is output to ros
            top_of_object_msg.header.frame_id = "world";
            top_of_object_msg.header.stamp = this->get_clock()->now();
            cluster2_surface_pub->publish(top_of_object_msg);


        }

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cluster0_sub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cluster1_sub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cluster2_sub;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster0_surface_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster1_surface_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster2_surface_pub;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr planar_surface_array_publisher_;


        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanarSurfaceMultipleObjects>());
  rclcpp::shutdown();
  return 0;
}