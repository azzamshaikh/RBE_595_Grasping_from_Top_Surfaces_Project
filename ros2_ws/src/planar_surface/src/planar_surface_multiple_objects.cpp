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
        PlanarSurfaceMultipleObjects(): Node("planar_grasping_multiple_objects")
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

            // add implementation

        }

        // Callback for each "cluster_" topic
        void cluster1_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            // Process the received point cloud data
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*msg,*cloud);

            // add implementation

        }
        
        // Callback for each "cluster_" topic
        void cluster2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            // Process the received point cloud data
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*msg,*cloud);

            // add implementation


        }

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cluster0_sub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cluster1_sub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cluster2_sub;


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