#include <chrono>
#include <memory>
#include <cmath>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <sstream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <array>
#include <vector>
#include <sensor_msgs/point_cloud2_iterator.hpp>

// added system headers and safer getenv handling
#include <sys/stat.h>
#include <sys/types.h>
#include <cerrno>
#include <cstring>
#include <cstdlib>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/msg/point_cloud2.hpp"


using namespace std::chrono_literals;

static std::string get_home_dir_safe()
{
  const char *home = std::getenv("HOME");
  if (home && std::strlen(home) > 0) {
    return std::string(home);
  }
  // fallback to current directory if HOME not set
  return std::string(".");
}

class RailSonarTF : public rclcpp::Node
{
public:
  RailSonarTF()
  : Node("rail_sonar_tf"),
    running_(true),
    rail_pos_(0.5),
    sonar_z_(-2.0),
    sonar_pitch_deg_(0.0),
    pending_save_(false),
    pending_rail_pos_m_(0.0)
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    serial_port_ = this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    baudrate_ = this->declare_parameter<int>("baudrate", 115200);

    // reserve a reasonable capacity to avoid frequent reallocations
    saved_scans_.reserve(16);
    saved_rail_positions_m_.reserve(16);

    if (!openSerial())
    {
      RCLCPP_WARN(this->get_logger(), "Failed to open serial port %s. Will continue and publish static example transforms.",
                  serial_port_.c_str());
    }
    else
    {
      reader_thread_ = std::thread(&RailSonarTF::serialReaderLoop, this);
    }

    timer_ = this->create_wall_timer(100ms, std::bind(&RailSonarTF::broadcast_transforms, this));

    // subscribe to pointclouds (sensor QoS)
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "sonar_points", rclcpp::SensorDataQoS(),
      std::bind(&RailSonarTF::cloudCallback, this, std::placeholders::_1));
  }

  ~RailSonarTF()
  {
    running_.store(false);
    if (reader_thread_.joinable()) reader_thread_.join();
    if (serial_fd_ >= 0) ::close(serial_fd_);
  }

private:
  void broadcast_transforms()
  {
    rclcpp::Time now = this->get_clock()->now();

    double rail_pos;
    double sonar_z;
    double pitch_deg;
    {
      std::lock_guard<std::mutex> lk(data_mtx_);
      rail_pos = rail_pos_;
      sonar_z = sonar_z_;
      pitch_deg = sonar_pitch_deg_;
    }

    geometry_msgs::msg::TransformStamped t1;
    t1.header.stamp = now;
    t1.header.frame_id = "world";
    t1.child_frame_id = "rail";
    t1.transform.translation.x = rail_pos;
    t1.transform.translation.y = 0.0;
    t1.transform.translation.z = 0.0;
    t1.transform.rotation.x = 0.0;
    t1.transform.rotation.y = 0.0;
    t1.transform.rotation.z = 0.0;
    t1.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(t1);

    geometry_msgs::msg::TransformStamped t2;
    t2.header.stamp = now;
    t2.header.frame_id = "rail";
    t2.child_frame_id = "sonar";
    t2.transform.translation.x = 0.0;
    t2.transform.translation.y = 0.0;
    t2.transform.translation.z = sonar_z;
    double pitch = -pitch_deg * M_PI / 180.0;
    t2.transform.rotation.x = 0.0;
    t2.transform.rotation.y = std::sin(pitch / 2.0);
    t2.transform.rotation.z = 0.0;
    t2.transform.rotation.w = std::cos(pitch / 2.0);
    tf_broadcaster_->sendTransform(t2);
  }

  bool openSerial()
  {
    serial_fd_ = ::open(serial_port_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) return false;

    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0)
    {
      ::close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }

    cfmakeraw(&tty);
    speed_t speed = B115200;
    switch (baudrate_)
    {
      case 9600: speed = B9600; break;
      case 19200: speed = B19200; break;
      case 38400: speed = B38400; break;
      case 57600: speed = B57600; break;
      case 115200: speed = B115200; break;
      default: speed = B115200; break;
    }
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
    {
      ::close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Opened serial port %s @ %d", serial_port_.c_str(), baudrate_);
    return true;
  }

  void serialReaderLoop()
  {
    std::string buffer;
    buffer.reserve(256);
    char tmp[128];
    while (running_.load())
    {
      if (serial_fd_ < 0) {
        std::this_thread::sleep_for(100ms);
        continue;
      }

      ssize_t n = ::read(serial_fd_, tmp, sizeof(tmp));
      if (n > 0)
      {
        buffer.append(tmp, tmp + n);
        size_t pos;
        while ((pos = buffer.find('\n')) != std::string::npos)
        {
          std::string line = buffer.substr(0, pos);
          buffer.erase(0, pos + 1);
          parseAndApplyLine(line);
        }
      }
      else
      {
        std::this_thread::sleep_for(50ms);
      }
    }
  }

  // Now accepts a single numeric value per line: rail_position (m)
  void parseAndApplyLine(const std::string &line)
  {
    std::istringstream ss(line);
    double v;
    if (ss >> v)
    {
      {
        std::lock_guard<std::mutex> lk(data_mtx_);
        rail_pos_ = v;
      }

      // arm one-shot save of next incoming pointcloud and remember rail pos for metadata
      {
        std::lock_guard<std::mutex> lk(save_mtx_);
        pending_rail_pos_m_ = v;
        pending_save_.store(true);
      }

      RCLCPP_DEBUG(this->get_logger(), "Updated rail_pos from serial: %.3f", rail_pos_);
    }
    else
    {
      RCLCPP_DEBUG(this->get_logger(), "Ignoring unparsable serial line: '%s'", line.c_str());
    }
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    // Only save when armed by parseAndApplyLine (one-shot)
    if (!pending_save_.load()) {
      return;
    }

    // consume the pending flag (one-shot)
    bool should_save = false;
    {
      std::lock_guard<std::mutex> lk(save_mtx_);
      if (pending_save_.load()) {
        pending_save_.store(false);
        should_save = true;
      }
    }
    if (!should_save) return;

    // snapshot rail pos for metadata
    double rail_pos_m = 0.0;
    {
      std::lock_guard<std::mutex> lk(save_mtx_);
      rail_pos_m = pending_rail_pos_m_;
    }

    // Reserve vector and copy points
    const size_t N = cloud->width * cloud->height;
    std::vector<std::array<float,3>> pts;
    pts.reserve(N);

    sensor_msgs::PointCloud2Iterator<float> it_x(*cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(*cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(*cloud, "z");
    for (size_t i = 0; i < N; ++i, ++it_x, ++it_y, ++it_z) {
      float x = *it_x;
      float y = *it_y;
      float z = *it_z;
      if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
        pts.push_back({x, y, z});
      } else {
        // skip invalid points (optional: push zeros instead)
      }
    }

    // enforce maximum buffer size
    if (saved_scans_.size() >= max_saved_scans_) {
      // drop oldest
      saved_scans_.erase(saved_scans_.begin());
      saved_rail_positions_m_.erase(saved_rail_positions_m_.begin());
    }

    saved_scans_.push_back(std::move(pts));
    saved_rail_positions_m_.push_back(rail_pos_m);

    RCLCPP_INFO(this->get_logger(), "Saved scan #%zu points=%zu rail_pos=%.3f m",
                saved_scans_.size()-1, saved_scans_.back().size(), rail_pos_m);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string serial_port_;
  int baudrate_;
  int serial_fd_ = -1;
  std::thread reader_thread_;
  std::atomic<bool> running_;
  std::mutex data_mtx_;

  double rail_pos_;
  double sonar_z_;
  double sonar_pitch_deg_;

  // storage: vector of scans; each scan is a vector of (x,y,z) arrays
  std::vector<std::vector<std::array<float,3>>> saved_scans_;
  std::vector<double> saved_rail_positions_m_; // same index -> rail pos
  size_t max_saved_scans_ = 200; // configurable limit to avoid OOM

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::atomic<bool> pending_save_;
  double pending_rail_pos_m_;
  std::mutex save_mtx_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RailSonarTF>());
  rclcpp::shutdown();
  return 0;
}
