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

        void clustering_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg){
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*cloud_msg,*cloud);
            
            // implement plane detection here
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(cloud);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(0.02);
            ec.setMinClusterSize(100);
            ec.setMaxClusterSize(25000);
            ec.setSearchMethod(tree);
            ec.setInputCloud(cloud);
            ec.extract(cluster_indices);

            std::vector<std::string> cluster_topic_names;
            auto cluster_topic_name_msg = clustering_itf::msg::StringArray();

            for(size_t id = 0; id < cluster_indices.size(); id++){
                //std::cout << "Adding topic" << std::endl;
                cluster_topic_names.push_back("/cluster_" + std::to_string(id));
            }
            
            cluster_topic_name_msg.string_array = cluster_topic_names;
            cluster_name_publisher_->publish(cluster_topic_name_msg);


            for(size_t id = 0; id < cluster_indices.size(); id++){
                std::string topic_name = cluster_topic_names[id];
                cluster_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name,10);
                auto& cluster = cluster_indices[id];

                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
                for (const auto& idx: cluster.indices){
                    cloud_cluster->push_back((*cloud)[idx]);
                }
                cloud_cluster->width = cloud_cluster->size();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;

                sensor_msgs::msg::PointCloud2 cluster_msg;
                pcl::toROSMsg(*cloud_cluster, cluster_msg);
                cluster_msg.header.frame_id = "world";
                cluster_msg.header.stamp = this->get_clock()->now();

                cluster_publisher_->publish(cluster_msg);
                
                //RCLCPP_INFO(this->get_logger(), "Published cluster %d", id);
            }

            // int cluster_id = 0;
            // for (const auto& cluster : cluster_indices){
            //     std::string topic_name = "/cluster_" + std::to_string(cluster_id);
            //     cluster_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name,10);
            //     // auto cluster_topic_name_msg = clustering_itf::msg::StringArray();
            //     // for (const auto& str : cluster_topic_names) {
            //     //     if (str == topic_name) {
            //     //         break;
            //     //     }
            //     //     else{
            //     //         cluster_topic_names.push_back(topic_name);

            //     //     }
            //     // }
            //     //cluster_topic_name_msg.string_array = cluster_topic_names;
            
            //     //cluster_publishers.push_back(this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name,10));
            //     //RCLCPP_INFO(this->get_logger(), "Publisher size %i", cluster_publishers.size());

            //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            //     for (const auto& idx: cluster.indices){
            //         cloud_cluster->push_back((*cloud)[idx]);
            //     }
            //     cloud_cluster->width = cloud_cluster->size();
            //     cloud_cluster->height = 1;
            //     cloud_cluster->is_dense = true;

            //     sensor_msgs::msg::PointCloud2 cluster_msg;
            //     pcl::toROSMsg(*cloud_cluster, cluster_msg);
            //     cluster_msg.header.frame_id = "world";
            //     cluster_msg.header.stamp = this->get_clock()->now();

            //     cluster_publisher_->publish(cluster_msg);
            //     //cluster_name_publisher_->publish(cluster_topic_name_msg);
            //     //cluster_publishers[cluster_id]->publish(cluster_msg); 
                
            //     RCLCPP_INFO(this->get_logger(), "Published cluster %d", cluster_id);
            //     cluster_id++;
            // }


            // sensor_msgs::msg::PointCloud2 top_of_object_msg;
            // pcl::toROSMsg(*top_of_object, top_of_object_msg); // make sure to change the reference input cloud that is being called here to the correct one that is output to ros
            // top_of_object_msg.header.frame_id = "world";
            // top_of_object_msg.header.stamp = this->get_clock()->now();
            // top_of_object_publisher_->publish(top_of_object_msg);

        }

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_publisher_;
        std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> cluster_publishers;
        
        rclcpp::Publisher<clustering_itf::msg::StringArray>::SharedPtr cluster_name_publisher_;
        
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Clustering>());
  rclcpp::shutdown();
  return 0;
}