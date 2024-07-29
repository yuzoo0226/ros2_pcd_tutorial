#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class PcdPublisher : public rclcpp::Node
{
public:
    PcdPublisher()
    : Node("pcd_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&PcdPublisher::publishPointCloud, this));
    }

private:
    void publishPointCloud()
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.width = 10000;
        cloud.height = 1;
        cloud.points.resize(cloud.width * cloud.height);

        for (auto & point : cloud.points) {
            point.x = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
            point.y = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
            point.z = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        }

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(cloud, output);
        output.header.frame_id = "base_link";
        output.header.stamp = this->now();

        publisher_->publish(output);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PcdPublisher>());
    rclcpp::shutdown();
    return 0;
}