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
    auto qos = rclcpp::SensorDataQoS();

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", qos,
      std::bind(&SonarPointCloudNode::scanCallback, this, std::placeholders::_1));

    auto cloud_qos = rclcpp::SensorDataQoS().reliable();
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sonar_points", cloud_qos);

  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const size_t n = msg->ranges.size();
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header = msg->header;
    cloud_msg.height = 1;
    cloud_msg.width = static_cast<uint32_t>(n);
    cloud_msg.is_dense = false;
    cloud_msg.is_bigendian = false;

    // set fields and allocate storage
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(n);

    sensor_msgs::PointCloud2Iterator<float> it_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud_msg, "z");

    float angle = msg->angle_min;
    for (size_t i = 0; i < n; ++i, angle += msg->angle_increment)
    {
      float r = msg->ranges[i];
      if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max)
      {
        *it_x = r * std::cos(angle);
        *it_y = r * std::sin(angle);
        *it_z = 0.0f;
      }
      else
      {
        *it_x = std::numeric_limits<float>::quiet_NaN();
        *it_y = std::numeric_limits<float>::quiet_NaN();
        *it_z = std::numeric_limits<float>::quiet_NaN();
      }
      ++it_x; ++it_y; ++it_z;
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