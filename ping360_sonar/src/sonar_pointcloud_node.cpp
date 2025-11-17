#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <ping360_sonar_msgs/msg/sonar_echo.hpp>
#include <array>
#include <cmath>
#include <limits>
#include <mutex>
#include <vector>

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
    center_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sonar_points_center", cloud_qos);
    intensity_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sonar_points_intensity", cloud_qos);
    intensity_sweep_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sonar_points_intensity_sweep", cloud_qos);

    echo_sub_ = this->create_subscription<ping360_sonar_msgs::msg::SonarEcho>(
      "scan_echo", qos,
      std::bind(&SonarPointCloudNode::echoCallback, this, std::placeholders::_1));

  }

private:
  // Patch limits lifted from the reference SolidWorks model:
  // area = R^2 * Δλ * (sin φ_max - sin φ_min) where the sonar beam spans 25° horizontally and 2° vertically.
  static constexpr double kVerticalSpanDeg = 25.0;
  static constexpr double kHorizontalSpanDeg = 2.0;
  static constexpr int kVerticalSegments = 11;
  static constexpr int kHorizontalSegments = 4;
  static constexpr size_t kPatchPointCount =
    static_cast<size_t>((kVerticalSegments + 1) * (kHorizontalSegments + 1));
  static constexpr size_t kIntensitySweepStride = 10;

  static double deg2rad(double degrees)
  {
    return degrees * M_PI / 180.0;
  }

  size_t appendSphericalPatch(float range,
                              float azimuth,
                              std::vector<std::array<float,3>> &points) const
  {
    // Shape definition copied from SolidWorks model: 25° vertical × 2° horizontal patch.
    const double vertical_span = deg2rad(kVerticalSpanDeg);
    const double horizontal_span = deg2rad(kHorizontalSpanDeg);

    // Keep the old point at the centre of the patch (symmetric limits around 0° elevation).
    const double elevation_min = -vertical_span / 2.0;
    const double elevation_max =  vertical_span / 2.0;
    const double az_min = azimuth - horizontal_span / 2.0;
    const double az_max = azimuth + horizontal_span / 2.0;

    for(int i = 0; i <= kVerticalSegments; ++i)
    {
      const double elev = elevation_min + (vertical_span * i / kVerticalSegments);
      for(int j = 0; j <= kHorizontalSegments; ++j)
      {
        const double az = az_min + (horizontal_span * j / kHorizontalSegments);
        const double cos_elev = std::cos(elev);
        const float x = static_cast<float>(range * cos_elev * std::cos(az));
        const float y = static_cast<float>(range * cos_elev * std::sin(az));
        const float z = static_cast<float>(range * std::sin(elev));
        points.push_back({x, y, z});
      }
    }
    return kPatchPointCount;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const size_t n = msg->ranges.size();
    std::vector<std::array<float,3>> patch_points;
    std::vector<std::array<float,3>> center_points;
    patch_points.reserve(n * kPatchPointCount);
    center_points.reserve(n);

    const auto nan = std::numeric_limits<float>::quiet_NaN();
    float angle = msg->angle_min;
    for (size_t i = 0; i < n; ++i, angle += msg->angle_increment)
    {
      const float r = msg->ranges[i];
      if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max)
      {
        appendSphericalPatch(r, angle, patch_points);
        const float x = r * std::cos(angle);
        const float y = r * std::sin(angle);
        center_points.push_back({x, y, 0.0f});
      }
      else
      {
        patch_points.push_back({nan, nan, nan});
        center_points.push_back({nan, nan, nan});
      }
    }

    cloud_pub_->publish(makeXYZCloud(msg->header, patch_points));
    center_pub_->publish(makeXYZCloud(msg->header, center_points));
    publishIntensitySweep(msg->header);
  }

  void echoCallback(const ping360_sonar_msgs::msg::SonarEcho::SharedPtr msg)
  {
    const auto sample_count = msg->intensities.size();
    if(sample_count == 0)
      return;

    std::vector<std::array<float,3>> points;
    std::vector<float> intensities;
    points.reserve(sample_count * kPatchPointCount);
    intensities.reserve(sample_count * kPatchPointCount);

    const float max_range = msg->range;
    const float angle = msg->angle;

    for(size_t idx = 0; idx < sample_count; ++idx)
    {
      const float r = static_cast<float>(idx + 1) * max_range / static_cast<float>(sample_count);
      const size_t added = appendSphericalPatch(r, angle, points);
      const auto intensity_value = static_cast<float>(msg->intensities[idx]);
      intensities.insert(intensities.end(), added, intensity_value);

      if(idx % kIntensitySweepStride == 0)
      {
        const float x = r * std::cos(angle);
        const float y = r * std::sin(angle);
        std::lock_guard<std::mutex> lock(intensity_mutex_);
        sweep_points_.push_back({x, y, 0.0f});
        sweep_values_.push_back(intensity_value);
      }
    }

    intensity_pub_->publish(makeXYZICloud(msg->header, points, intensities));
  }

  sensor_msgs::msg::PointCloud2 makeXYZCloud(
      const std_msgs::msg::Header &header,
      const std::vector<std::array<float,3>> &points) const
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(points.size());
    cloud.is_dense = false;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
    for (const auto &pt : points)
    {
      *it_x = pt[0];
      *it_y = pt[1];
      *it_z = pt[2];
      ++it_x; ++it_y; ++it_z;
    }
    return cloud;
  }

  sensor_msgs::msg::PointCloud2 makeXYZICloud(
      const std_msgs::msg::Header &header,
      const std::vector<std::array<float,3>> &points,
      const std::vector<float> &intensities) const
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(points.size());
    cloud.is_dense = false;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> it_i(cloud, "intensity");
    for (size_t idx = 0; idx < points.size(); ++idx)
    {
      *it_x = points[idx][0];
      *it_y = points[idx][1];
      *it_z = points[idx][2];
      *it_i = intensities[idx];
      ++it_x; ++it_y; ++it_z; ++it_i;
    }
    return cloud;
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<ping360_sonar_msgs::msg::SonarEcho>::SharedPtr echo_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr center_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr intensity_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr intensity_sweep_pub_;

  std::mutex intensity_mutex_;
  std::vector<std::array<float,3>> sweep_points_;
  std::vector<float> sweep_values_;

  void publishIntensitySweep(const std_msgs::msg::Header &header)
  {
    std::vector<std::array<float,3>> points;
    std::vector<float> values;
    {
      std::lock_guard<std::mutex> lock(intensity_mutex_);
      if(sweep_points_.empty())
        return;
      points.swap(sweep_points_);
      values.swap(sweep_values_);
    }

    intensity_sweep_pub_->publish(makeXYZICloud(header, points, values));
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SonarPointCloudNode>());
  rclcpp::shutdown();
  return 0;
}
