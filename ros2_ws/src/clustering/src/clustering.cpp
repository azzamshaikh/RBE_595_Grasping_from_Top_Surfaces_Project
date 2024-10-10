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
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
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
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_clusters.h>
#include "clustering_itf/msg/string_array.hpp"

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Clustering : public rclcpp::Node
{
    public:
        Clustering(): Node("clustering")
        {

            pointcloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/object_pointcloud_data", 10,
                std::bind(&Clustering::clustering_callback, this, std::placeholders::_1));
            cluster_name_publisher_ = this->create_publisher<clustering_itf::msg::StringArray>("/cluster_topic_names", 10);
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }

    private:
        // Callback function for clustering the input point cloud
        void clustering_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg){
            // Convert the incoming ROS point cloud message to a PCL point cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*cloud_msg,*cloud);
            
            // Create a KD-tree for efficient neighbor search
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(cloud);
            
            // Store the indices of the clustered points
            std::vector<pcl::PointIndices> cluster_indices;

            // Set up Euclidean clustering algorithm
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(0.02);
            ec.setMinClusterSize(100);
            ec.setMaxClusterSize(25000);
            ec.setSearchMethod(tree);
            ec.setInputCloud(cloud);
            ec.extract(cluster_indices);

            // Prepare a vector to store the names of topics for each detected cluster
            std::vector<std::string> cluster_topic_names;
            auto cluster_topic_name_msg = clustering_itf::msg::StringArray();
            cluster_publishers.clear();

            // For each detected cluster, create a new ROS topic and publisher
            for(size_t id = 0; id < cluster_indices.size(); id++){
                cluster_topic_names.push_back("/cluster_" + std::to_string(id));
                cluster_publishers.push_back(this->create_publisher<sensor_msgs::msg::PointCloud2>("/cluster_" + std::to_string(id),10));
            }
            
            // Store the cluster topic names in the message and publish them
            cluster_topic_name_msg.string_array = cluster_topic_names;
            cluster_name_publisher_->publish(cluster_topic_name_msg);

            // Loop through each cluster and publish the individual cluster point cloud to its topic
            for(size_t id = 0; id < cluster_indices.size(); id++){
                std::string topic_name = cluster_topic_names[id];  // Get the topic name for the current cluster
                auto& cluster = cluster_indices[id];               // Get the point indices  for the current cluster 

                // Create a new point cloud to store the points belonging to the current cluster
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
                for (const auto& idx: cluster.indices){
                    cloud_cluster->push_back((*cloud)[idx]);
                }
                
                // Set the dimensions and density of the point cloud (height = 1 since it's organized as a 1D point cloud)
                cloud_cluster->width = cloud_cluster->size();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;

                // Convert the PCL point cloud to a ROS message
                sensor_msgs::msg::PointCloud2 cluster_msg;
                pcl::toROSMsg(*cloud_cluster, cluster_msg);
                cluster_msg.header.frame_id = "world";
                cluster_msg.header.stamp = this->get_clock()->now();

                // Publish the point cloud of the current cluster to its respective topic
                cluster_publishers[id]->publish(cluster_msg);
            }

        }

        // Setup subscribers and publishers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_publisher_;
        std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> cluster_publishers;
        rclcpp::Publisher<clustering_itf::msg::StringArray>::SharedPtr cluster_name_publisher_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char * argv[])
{
  // Initialize the ROS2 system
  rclcpp::init(argc, argv);
  // Create an instance of the Denoise node and start spinning to process callbacks
  rclcpp::spin(std::make_shared<Clustering>());
  // Shutdown the ROS2 system when the node is done
  rclcpp::shutdown();
  return 0;
}