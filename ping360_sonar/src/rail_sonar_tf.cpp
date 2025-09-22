#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class RailSonarTF : public rclcpp::Node
{
public:
  RailSonarTF()
  : Node("rail_sonar_tf")
  {
    // Create TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Timer to broadcast transforms at 10 Hz
    timer_ = this->create_wall_timer(100ms, std::bind(&RailSonarTF::broadcast_transforms, this));
  }

private:
  void broadcast_transforms()
  {
    rclcpp::Time now = this->get_clock()->now();

    // ---- world -> rail ----
    geometry_msgs::msg::TransformStamped t1;
    t1.header.stamp = now;
    t1.header.frame_id = "world";
    t1.child_frame_id = "rail";

    t1.transform.translation.x = get_rail_position();
    t1.transform.translation.y = 0.0;
    t1.transform.translation.z = 0.0;

    t1.transform.rotation.x = 0.0;
    t1.transform.rotation.y = 0.0;
    t1.transform.rotation.z = 0.0;
    t1.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(t1);

    // ---- rail -> sonar ----
    geometry_msgs::msg::TransformStamped t2;
    t2.header.stamp = now;
    t2.header.frame_id = "rail";
    t2.child_frame_id = "sonar";

    // Example: sonar mounted 5 cm above rail
    t2.transform.translation.x = 0.0;
    t2.transform.translation.y = 0.0;
    t2.transform.translation.z = -2.0;

    // Example: sonar tilted -30 degrees around Y axis
    double pitch = 30.0 * M_PI / 180.0;
    t2.transform.rotation.x = 0.0;
    t2.transform.rotation.y = std::sin(pitch / 2.0);
    t2.transform.rotation.z = 0.0;
    t2.transform.rotation.w = std::cos(pitch / 2.0);

    tf_broadcaster_->sendTransform(t2);
  }

  // Replace with encoder feedback
  double get_rail_position()
  {
    return 0.5;  // Example: 0.5 m along x-axis
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RailSonarTF>());
  rclcpp::shutdown();
  return 0;
}
