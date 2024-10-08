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

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MajorPlaneDetection : public rclcpp::Node
{
    public:
        MajorPlaneDetection(): Node("plane_detection")
        {

            pointcloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/denoised_pointcloud_data", 10,
                std::bind(&MajorPlaneDetection::plane_detection_callback, this, std::placeholders::_1));

            object_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/object_pointcloud_data", 10);
            table_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/table_pointcloud_data", 10);

            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }

    private:
        pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointsAboveTable(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
            const pcl::ModelCoefficients::Ptr &plane_coefficients,
            float threshold = 0.0000001)
        {
            float a = plane_coefficients->values[0];
            float b = plane_coefficients->values[1];
            float c = plane_coefficients->values[2];
            float d = plane_coefficients->values[3];

            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

            for(const auto &point : cloud->points)
            {
                // Calculate the distance of the poitn from the plane
                float distance = a*point.x + b*point.y + c*point.z + d;

                // keep only points that are above the plane (distance > threshold)
                if(distance > threshold){
                    filtered_cloud->points.push_back(point);
                }
            }

            filtered_cloud->width = filtered_cloud->points.size();
            filtered_cloud->height = 1;
            filtered_cloud->is_dense = true;
            
            return filtered_cloud;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointsByHeight(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, 
            float min_height)
        {
            pcl::PassThrough<pcl::PointXYZ> pass;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

            // Set filter to the z-axis (height)
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("y");
            pass.setFilterLimits(min_height, FLT_MAX);  // Keep points above min_height
            pass.filter(*cloud_filtered);

            return cloud_filtered;
        }

        void plane_detection_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg){
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*cloud_msg,*cloud);
            
            // implement plane detection here
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setDistanceThreshold (0.01);
            seg.setInputCloud (cloud);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0)
            {
                PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
                return;
            }
            //std::cerr << "Model coefficients: " << coefficients->values[0] << " " << coefficients->values[1] << " "<< coefficients->values[2] << " " << coefficients->values[3] << std::endl;
            //std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
            // assign the inliers to a new point cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr table_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            for (size_t i = 0; i < inliers->indices.size (); ++i)
                table_cloud->push_back(cloud->points[inliers->indices[i]]);

            pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);

            std::set<int> inlier_set(inliers->indices.begin(), inliers->indices.end());

            for (size_t i = 0; i < cloud->points.size(); ++i){
                if(inlier_set.find(i) == inlier_set.end()){
                    outlier_cloud->push_back(cloud->points[i]);
                }
            }



            sensor_msgs::msg::PointCloud2 table_cloud_msg;
            pcl::toROSMsg(*table_cloud, table_cloud_msg); // make sure to change the reference input cloud that is being called here to the correct one that is output to ros
            table_cloud_msg.header.frame_id = "world";
            table_cloud_msg.header.stamp = this->get_clock()->now();
            table_pointcloud_publisher_->publish(table_cloud_msg);

            //pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = filterPointsAboveTable(cloud, coefficients);
            // pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = filterPointsByHeight(cloud,-coefficients->values[3]);

            // Convert denoised PLC point cloud abck to sensor_msgs::PointCloud2
            sensor_msgs::msg::PointCloud2 object_cloud_msg;
            pcl::toROSMsg(*outlier_cloud, object_cloud_msg); // make sure to change the reference input cloud that is being called here to the correct one that is output to ros
            object_cloud_msg.header.frame_id = "world";
            object_cloud_msg.header.stamp = this->get_clock()->now();
            object_pointcloud_publisher_->publish(object_cloud_msg);

        }

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr object_pointcloud_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr table_pointcloud_publisher_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MajorPlaneDetection>());
  rclcpp::shutdown();
  return 0;
}