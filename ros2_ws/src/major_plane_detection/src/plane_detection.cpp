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

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MajorPlaneDetection : public rclcpp::Node
{
    public:
        MajorPlaneDetection(): Node("plane_detection")
        {

            pointcloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/transformed_pointcloud_data", 10,
                std::bind(&MajorPlaneDetection::plane_detection_callback, this, std::placeholders::_1));

            object_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/object_pointcloud_data", 10);

            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }

    private:
        void plane_detection_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg){
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*cloud_msg,*cloud);
            
            // implement plane detection here
            

            // Convert denoised PLC point cloud abck to sensor_msgs::PointCloud2
            sensor_msgs::msg::PointCloud2 object_cloud_msg;
            pcl::toROSMsg(*cloud, object_cloud_msg); // make sure to change the reference input cloud that is being called here to the correct one that is output to ros
            object_cloud_msg.header.frame_id = "world";
            object_cloud_msg.header.stamp = this->get_clock()->now();

            object_pointcloud_publisher_->publish(object_cloud_msg);
        }

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr object_pointcloud_publisher_;
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