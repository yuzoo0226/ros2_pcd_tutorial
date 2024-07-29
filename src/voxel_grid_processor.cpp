#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

class VoxelGridProcessor : public rclcpp::Node
{
public:
    VoxelGridProcessor()
    : Node("voxel_grid_processor")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "point_cloud", 10, std::bind(&VoxelGridProcessor::pointCloudCallback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_grid", 10);

        this->declare_parameter<float>("leaf_size_x", 0.1);
        this->declare_parameter<float>("leaf_size_y", 0.1);
        this->declare_parameter<float>("leaf_size_z", 0.1);

        leaf_size_x_ = this->get_parameter("leaf_size_x").as_double();
        leaf_size_y_ = this->get_parameter("leaf_size_y").as_double();
        leaf_size_z_ = this->get_parameter("leaf_size_z").as_double();
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(static_cast<float>(leaf_size_x_), static_cast<float>(leaf_size_y_), static_cast<float>(leaf_size_z_));
        sor.filter(*cloud_filtered);

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_filtered, output);
        output.header.frame_id = msg->header.frame_id;
        output.header.stamp = this->now();

        publisher_->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    double leaf_size_x_;
    double leaf_size_y_;
    double leaf_size_z_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VoxelGridProcessor>());
    rclcpp::shutdown();
    return 0;
}