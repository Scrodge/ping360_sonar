#include <ping360_sonar/ping360_node.h>
#include <ping360_sonar/sector.h>
#include <ping-message-common.h>
#include <ping-message-ping360.h>
#include <cmath>  // >>> ADDED FOR SONAR PITCH <<<

using namespace std::chrono_literals;
using namespace ping360_sonar;
using std::string;
using std::vector;

Ping360Sonar::Ping360Sonar(rclcpp::NodeOptions options)
  : Node("ping360", options)
{ 
  // bounded parameters declared with descriptors so rqt can create sliders
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(0);
    range.set__to_value(2);
    range.set__step(1);
    desc.integer_range = {range};
    declare_parameter<int>("gain", 0, desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(650);
    range.set__to_value(850);
    range.set__step(1);
    desc.integer_range = {range};
    declare_parameter<int>("frequency", 740, desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(1);
    range.set__to_value(50);
    range.set__step(1);
    desc.integer_range = {range};
    declare_parameter<int>("range_max", 2, desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(60);
    range.set__to_value(360);
    range.set__step(1);
    desc.integer_range = {range};
    declare_parameter<int>("angle_sector", 360, desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(1);
    range.set__to_value(20);
    range.set__step(1);
    desc.integer_range = {range};
    declare_parameter<int>("angle_step", 1, desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(100);
    range.set__to_value(1000);
    range.set__step(2);
    desc.integer_range = {range};
    declare_parameter<int>("image_size", 300, desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(1);
    range.set__to_value(255);
    range.set__step(1);
    desc.integer_range = {range};
    declare_parameter<int>("scan_threshold", 200, desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(1450);
    range.set__to_value(1550);
    range.set__step(1);
    desc.integer_range = {range};
    declare_parameter<int>("speed_of_sound", 1500, desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    rcl_interfaces::msg::FloatingPointRange fr;
    fr.set__from_value(50);
    fr.set__to_value(2000);
    fr.set__step(1);
    desc.floating_point_range = {fr};
    declare_parameter<int>("image_rate", 100, desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(0);
    range.set__to_value(20000);
    range.set__step(1);
    desc.integer_range = {range};
    declare_parameter<int>("sonar_timeout", 8000, desc);
  }

  // boolean/unbounded params
  declare_parameter<bool>("publish_image", true);
  declare_parameter<bool>("publish_scan", true);
  declare_parameter<bool>("publish_echo", true);

  // >>> ADDED FOR SONAR PITCH PARAMETER <<<
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "Sonar tilt (pitch) in degrees. Positive = tilt downward.";
    declare_parameter<int>("sonar_pitch_deg", 0, desc);
  }
  // >>> END ADDED <<<

  // frame string
  declare_parameter<std::string>("frame", "sonar");
  const auto frame{get_parameter("frame").as_string()};
  image.header.set__frame_id(frame);
  image.set__encoding("mono8");
  image.set__is_bigendian(0);
  scan.header.set__frame_id(frame);
  scan.set__range_min(0.5);
  echo.header.set__frame_id(frame);
 
  // ROS interface
  configureFromParams();
 
  const auto image_rate_ms{get_parameter("image_rate").as_int()};
  image_timer = this->create_wall_timer(std::chrono::milliseconds(image_rate_ms),
                                         [this](){publishImage();});
 
  param_change = add_on_set_parameters_callback(
                    std::bind(&Ping360Sonar::parametersCallback, this, std::placeholders::_1));
}

Ping360Sonar::IntParams Ping360Sonar::updatedParams(const std::vector<rclcpp::Parameter> &new_params) const
{
  using ParamType = rclcpp::ParameterType;
  const std::map<ParamType,vector<string>> mutable_params{
    {ParamType::PARAMETER_INTEGER,{"gain","frequency","range_max",
                                   "angle_sector","angle_step",
                                   "speed_of_sound","image_size","scan_threshold","sonar_timeout"}},
    {ParamType::PARAMETER_BOOL, {"publish_image","publish_scan","publish_echo"}}};

  IntParams mapping;
  for(const auto &[type,names]: mutable_params)
  {
    const auto params{get_parameters(names)};
    if(type == ParamType::PARAMETER_INTEGER)
    {
      for(auto &param: params)
        mapping[param.get_name()] = param.as_int();
    }
    else
    {
      for(auto &param: params)
        mapping[param.get_name()] = param.as_bool();
    }
  }
  for(auto &param: new_params)
  {
    if(param.get_type() == ParamType::PARAMETER_BOOL)
      mapping[param.get_name()] = param.as_bool();
    else if(param.get_type() == ParamType::PARAMETER_INTEGER)
      mapping[param.get_name()] = param.as_int();
  }

  return mapping;
}

SetParametersResult Ping360Sonar::parametersCallback(const vector<rclcpp::Parameter> &parameters)
{
  configureFromParams(parameters);
  return SetParametersResult().set__successful(true);
}

void Ping360Sonar::initPublishers(bool image, bool scan, bool echo)
{
#ifdef PING360_PUBLISH_RELIABLE
  const auto qos{rclcpp::QoS(5)};
#else
  const auto qos{rclcpp::SensorDataQoS()};
#endif

  publish_echo = echo;
  publish_image = image;
  publish_scan = scan;

  if(publish_image && image_pub.getTopic().empty())
    image_pub = image_transport::create_publisher(this, "scan_image");

  if(publish_echo && echo_pub == nullptr)
    echo_pub = create_publisher<ping360_sonar_msgs::msg::SonarEcho>("scan_echo", qos);

  if(publish_scan && scan_pub == nullptr)
    scan_pub = create_publisher<sensor_msgs::msg::LaserScan>("scan", qos);
}

void Ping360Sonar::configureFromParams(const vector<rclcpp::Parameter> &new_params)
{
  const auto params{updatedParams(new_params)};

  const auto [angle_sector, step] = sonar.configureAngles(params.at("angle_sector"),
      params.at("angle_step"),
      params.at("publish_scan")); {}

  if(angle_sector != params.at("angle_sector") || step != params.at("angle_step"))
  {
    RCLCPP_INFO(get_logger(),
                "Due to sonar using gradians, sector is %i (requested %i) and step is %i (requested %i)",
                angle_sector, params.at("angle_sector"), step, params.at("angle_step"));
  }

  initPublishers(params.at("publish_image"),
                 params.at("publish_scan"),
                 params.at("publish_echo"));

  sonar.configureTransducer(params.at("gain"),
                            params.at("frequency"),
                            params.at("speed_of_sound"),
                            params.at("range_max"));
  sonar.setTimeout(params.at("sonar_timeout"));

  echo.set__gain(params.at("gain"));
  echo.set__range(params.at("range_max"));
  echo.set__speed_of_sound(params.at("speed_of_sound"));
  echo.set__number_of_samples(sonar.samples());
  echo.set__transmit_frequency(params.at("frequency"));

  scan.set__range_max(params.at("range_max"));
  scan.set__time_increment(sonar.transmitDuration());
  scan.set__angle_max(sonar.angleMax());
  scan.set__angle_min(sonar.angleMin());
  scan.set__angle_increment(sonar.angleStep());

  const int size{params.at("image_size")};
  if(size != static_cast<int>(image.step) ||
     std::any_of(new_params.begin(), new_params.end(),
                 [](const auto &param){return param.get_name() == "angle_sector";}))
  {
    image.data.resize(size*size);
    std::fill(image.data.begin(), image.data.end(), 0);
    image.height = image.width = image.step = size;
  }

  sector.configure(sonar.samples(), size/2);
  scan_threshold = params.at("scan_threshold");
}


void Ping360Sonar::publishEcho(const rclcpp::Time &now)
{
  const auto [data, length] = sonar.intensities(); {}
  echo.angle = sonar.currentAngle();
  echo.intensities.resize(length);
  std::copy(data, data+length, echo.intensities.begin());
  echo.header.set__stamp(now);
  echo_pub->publish(echo);
}

void Ping360Sonar::publishScan(const rclcpp::Time &now, bool end_turn)
{
  scan.ranges.resize(sonar.angleCount());
  scan.intensities.resize(sonar.angleCount());

  const auto angle{sonar.angleIndex()};
  auto &this_range = scan.ranges[angle] = 0;
  auto &this_intensity = scan.intensities[angle] = 0;

  const auto [data, length] = sonar.intensities(); {}
  for(int index=0; index<length; index++)
  {
    if(data[index] >= scan_threshold)
    {
      if(const auto range{sonar.rangeFrom(index)};
         range >= scan.range_min && range < scan.range_max)
      {
        this_range = range;
        this_intensity = data[index]/255.f;
        break;
      }
    }
  }

  // >>> ADDED FOR SONAR PITCH <<<
  // int pitch_deg_int = get_parameter("sonar_pitch_deg").as_int();
  // double pitch_deg = static_cast<double>(pitch_deg_int);
  // double pitch_rad = pitch_deg * M_PI / 180.0;

  // // double pitch_deg = get_parameter("sonar_pitch_deg").as_double();
  // // double pitch_rad = pitch_deg * M_PI / 180.0;
  // RCLCPP_DEBUG(get_logger(), "Sonar tilt applied: %.2f deg (%.3f rad)", pitch_deg, pitch_rad);
  // scan.header.frame_id = "sonar_tilted";  // optional: mark tilted frame
  // >>> END ADDED <<<

  if(end_turn)
  {
    if(!sonar.fullScan())
    {
      if(sonar.angleStep() < 0)
      {
        scan.set__angle_max(sonar.angleMax());
        scan.set__angle_min(sonar.angleMin());
      }
      else
      {
        scan.set__angle_max(sonar.angleMin());
        scan.set__angle_min(sonar.angleMax());
      }
      scan.set__angle_increment(-sonar.angleStep());
      scan.angle_max -= scan.angle_increment;
    }
    scan.header.set__stamp(now);
    scan_pub->publish(scan);
  }
}

void Ping360Sonar::refreshImage()
{
  const auto [data, length] = sonar.intensities(); {}
  if(length == 0) return;
  const auto half_size{image.step/2};

  sector.init(sonar.currentAngle(), fabs(sonar.angleStep()));
  int x{}, y{}, index{};

  while(sector.nextPoint(x, y, index))
  {
    if(index < length)
      image.data[half_size-y + image.step*(half_size-x)] = data[index];
  }
}

void Ping360Sonar::refresh()
{
  const auto &[valid, end_turn] = sonar.read(); {}
  
  if(!valid)
  {
    RCLCPP_WARN(get_logger(), "Cannot communicate with sonar");
    return;
  }

  const auto now{this->now()};
  if(publish_echo && echo_pub->get_subscription_count())
    publishEcho(now);

  if(publish_image)
    refreshImage();

  if(publish_scan && scan_pub->get_subscription_count())
    publishScan(now, end_turn);
}

void Ping360Sonar::publishImage()
{
  if(publish_image)
  {
    image.header.set__stamp(now());
    image_pub.publish(image);
  }
}
