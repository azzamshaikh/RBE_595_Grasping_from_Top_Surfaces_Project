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
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class PlanarSurfaceMultipleObjects : public rclcpp::Node
{
    public:
        PlanarSurfaceMultipleObjects(): Node("planar_grasping")
        {
            auto cluster_topics = find_cluster_topics();
            
            for (const auto& topic_name : cluster_topics)
            {
                current_topic_name = topic_name;

                RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", current_topic_name.c_str());
                auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    topic_name, 10,
                    std::bind(&PlanarSurfaceMultipleObjects::cluster_callback, this, std::placeholders::_1));
                subscriptions_.push_back(sub);
            }

            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }

        //     // Function to process all stored point clouds
        // void process_clusters()
        // {
        //     for (const auto& cloud : point_clouds_)
        //     {
        //         if (cloud->points.empty())
        //         {
        //             RCLCPP_WARN(this->get_logger(), "Empty point cloud detected, skipping.");
        //             continue;
        //         }
                
        //         // Your processing code goes here
        //         RCLCPP_INFO(this->get_logger(), "Processing a cluster with %zu points", cloud->points.size());

        //         // Example: you can do point cloud processing like filtering or segmentation here
        //         // pcl::PassThrough<pcl::PointXYZ> pass;
        //         // pass.setInputCloud(cloud);
        //         // pass.setFilterFieldName("z");
        //         // pass.setFilterLimits(0.0, 1.0);
        //         // pass.filter(*cloud);
        //     }
        // }

    private:
        std::vector<std::string> find_cluster_topics()
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for topics to be available.");
            std::vector<std::string> cluster_topics;

            // Get all topic names and their types
            auto topics = this->get_topic_names_and_types();

            // Loop through each topic and find those starting with "cluster_"
            for (const auto& topic : topics)
            {

                const std::string& topic_name = topic.first;
                if (topic_name.find("cluster_") != std::string::npos) // Check if "cluster_" is at the beginning of the topic name
                {
                    cluster_topics.push_back(topic_name);
                    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", topic_name);
                }
            }
            RCLCPP_INFO(this->get_logger(), "Topics are available.");
            return cluster_topics;
        }

        // Callback for each "cluster_" topic
        void cluster_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            //RCLCPP_INFO(this->get_logger(), "Received data from: %s", cur.c_str());
            // Process the received point cloud data
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*msg,*cloud);

            std::string top_of_object_topic = current_topic_name + "_top_of_object";
            std::string convex_hull = current_topic_name + "_convex";

            top_of_object_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(top_of_object_topic,10);
            concave_hull_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(convex_hull,10);

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
            //std::cerr << "Model coefficients: " << coefficients->values[0] << " " << coefficients->values[1] << " "<< coefficients->values[2] << " " << coefficients->values[3] << std::endl;
            //std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

            pcl::PointCloud<pcl::PointXYZ>::Ptr top_of_object(new pcl::PointCloud<pcl::PointXYZ>);
            for (size_t i = 0; i < inliers->indices.size (); ++i)
                top_of_object->push_back(cloud->points[inliers->indices[i]]);

            sensor_msgs::msg::PointCloud2 top_of_object_msg;
            pcl::toROSMsg(*top_of_object, top_of_object_msg); // make sure to change the reference input cloud that is being called here to the correct one that is output to ros
            top_of_object_msg.header.frame_id = "world";
            top_of_object_msg.header.stamp = this->get_clock()->now();

            top_of_object_publisher_->publish(top_of_object_msg);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
            
            pcl::ProjectInliers<pcl::PointXYZ> proj;
            proj.setModelType(pcl::SACMODEL_PLANE);
            proj.setInputCloud(top_of_object);
            proj.setModelCoefficients(coefficients);
            proj.filter(*cloud_projected);

            float max_z = coefficients->values[3];
            float z_threshold = 0.00001;

            pcl::PointCloud<pcl::PointXYZ>::Ptr top_surface_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto& point : cloud_projected->points){
                if(point.z >= (max_z - z_threshold)){
                    top_surface_cloud->push_back(point);
                }
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ConvexHull<pcl::PointXYZ> chull;

            chull.setInputCloud(top_surface_cloud);
            chull.setComputeAreaVolume(true);
            chull.reconstruct(*cloud_hull);

            // Convert denoised PLC point cloud abck to sensor_msgs::PointCloud2
            sensor_msgs::msg::PointCloud2 convex_hull_cloud_msg;
            pcl::toROSMsg(*cloud_hull, convex_hull_cloud_msg); // make sure to change the reference input cloud that is being called here to the correct one that is output to ros
            convex_hull_cloud_msg.header.frame_id = "world";
            convex_hull_cloud_msg.header.stamp = this->get_clock()->now();
            concave_hull_publisher_->publish(convex_hull_cloud_msg);

        }

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;

        // A vector to store all the subscribers
        std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subscriptions_;
        // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> point_clouds_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr top_of_object_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr concave_hull_publisher_;
        std::string current_topic_name;

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