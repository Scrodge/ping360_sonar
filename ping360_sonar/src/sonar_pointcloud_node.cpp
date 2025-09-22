#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <limits>

class SonarPointCloudNode : public rclcpp::Node
{
public:
  SonarPointCloudNode()
  : Node("sonar_pointcloud_node")
  {
    // Use sensor data QoS for compatibility with LaserScan publisher
    auto qos = rclcpp::SensorDataQoS();
    
    //new one
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", qos,
      std::bind(&SonarPointCloudNode::scanCallback, this, std::placeholders::_1));

    //old one
    // Subscribe to the LaserScan topic published by ping360_sonar
    // scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    //   "scan", 10,
    //   std::bind(&SonarPointCloudNode::scanCallback, this, std::placeholders::_1));

    // Publisher for the PointCloud2 message
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sonar_points", 10);
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    size_t n = msg->ranges.size();
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header = msg->header;
    cloud_msg.height = 1;
    cloud_msg.width = n;
    cloud_msg.is_dense = false;
    cloud_msg.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    float angle = msg->angle_min;
    for (size_t i = 0; i < n; ++i, angle += msg->angle_increment)
    {
      float r = msg->ranges[i];
      if (r > msg->range_min && r < msg->range_max && std::isfinite(r))
      {
        *iter_x = r * std::cos(angle);
        *iter_y = r * std::sin(angle);
        *iter_z = 0.0f; // All points in XY plane
      }
      else
      {
        *iter_x = std::numeric_limits<float>::quiet_NaN();
        *iter_y = std::numeric_limits<float>::quiet_NaN();
        *iter_z = std::numeric_limits<float>::quiet_NaN();
      }
      ++iter_x; ++iter_y; ++iter_z;
    }

    cloud_pub_->publish(cloud_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SonarPointCloudNode>());
  rclcpp::shutdown();
  return 0;
}