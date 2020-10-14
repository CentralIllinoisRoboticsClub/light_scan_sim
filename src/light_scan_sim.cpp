/**
 * light_scan_sim light_scan_sim.cpp
 * @brief Monitor map and tf data, publish simulated laser scan
 *
 * @copyright 2017 Joseph Duchesne
 * @author Joseph Duchesne
 * 
 */

#include "light_scan_sim/light_scan_sim.h"
#include <math.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/create_timer_ros.h>

using std::placeholders::_1;
using std::placeholders::_2;

/**
 * @brief Initialize light scan sim class
 *
 * @param node The ros node handle
 */
LightScanSim::LightScanSim(const rclcpp::NodeOptions& node_options) :
Node("light_scan_sim", node_options), tf_broadcaster_(this)
{
  // https://github.com/ros-planning/navigation2/blob/foxy-devel/nav2_amcl/src/amcl_node.cpp
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(),
      get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  tf_buffer_->lookupTransform("laser", "odom", rclcpp::Time(0), rclcpp::Duration(10.0));
  tf_buffer_->lookupTransform("base_link", "odom", rclcpp::Time(0), rclcpp::Duration(10.0));

  declare_parameter("freq_hz", 20.0);
  freq_hz_ = get_parameter("freq_hz").as_double();

  declare_parameter("range_min", 1.0);
  declare_parameter("range_max", 20.0);
  declare_parameter("angle_min", -M_PI_2);
  declare_parameter("angle_max", M_PI_2);
  declare_parameter("angle_increment", 0.01);
  declare_parameter("range_noise", 0.01);
  // Load settings
  ray_cast_ = std::make_shared<RayCast>(get_parameter("range_min").as_double(),
                                        get_parameter("range_max").as_double(),
                                        get_parameter("angle_min").as_double(),
                                        get_parameter("angle_max").as_double(),
                                        get_parameter("angle_increment").as_double(),
                                        get_parameter("range_noise").as_double() );

  declare_parameter("map/topic", "map");
  declare_parameter("map/materials_topic", "map_materials");
  declare_parameter("map/segments_topic", "map_segments");
  declare_parameter("laser/topic", "scan");
  declare_parameter("map/image_frame", "map");
  declare_parameter("laser/frame", "laser");

  map_topic_ = get_parameter("map/topic").as_string();
  materials_topic_ = get_parameter("map/materials_topic").as_string();
  segments_topic_ = get_parameter("map/segments_topic").as_string();
  laser_topic_ = get_parameter("laser/topic").as_string();
  image_frame_ = get_parameter("map/image_frame").as_string();
  laser_frame_ = get_parameter("laser/frame").as_string();

  // Subscribe / Publish
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic_, 1, std::bind(&LightScanSim::MapCallback, this, _1) );
  materials_sub_ = create_subscription<light_scan_sim::msg::MaterialList>(materials_topic_, 1, std::bind(&LightScanSim::MaterialsCallback, this, _1) );
  segments_sub_ = create_subscription<light_scan_sim::msg::SegmentList>(segments_topic_, 1, std::bind(&LightScanSim::SegmentsCallback, this, _1) );
  laser_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(laser_topic_, 1);
}

/**
 * @brief Recieve the subscribed map and process its data
 *
 * @param grid The map occupancy grid
 */ 
void LightScanSim::MapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr grid)
{
  map_ = *grid;  // Copy the entire message
  
  // Convert OccupancyGrid to cv::Mat, uint8_t
  cv::Mat map_mat = cv::Mat(map_.info.height, map_.info.width,
                            CV_8UC1, map_.data.data());
  // Set unknown space (255) to free space (0)
  // 4 = threshold to zero, inverted
  // See: http://docs.opencv.org/3.1.0/db/d8e/tutorial_threshold.html
  cv::threshold(map_mat, map_mat, 254, 255, 4); 

  // Update map
  ray_cast_->SetMap(map_mat, map_.info.resolution, map_.info.origin.position.x, map_.info.origin.position.y);
  
  // Create transform from map tf to image tf
  map_to_image_.setOrigin(tf2::Vector3(map_.info.origin.position.x,
                                      map_.info.origin.position.y,
                                      map_.info.origin.position.z));
  // Image is in standard right hand orientation
  tf2::Quaternion quat;
  quat.setRPY( 0, 0, 0 );
  map_to_image_.setRotation(quat);

  map_loaded_ = true;
  if (map_loaded_ && segments_loaded_ && materials_loaded_) {
    ray_cast_->SetSegments(segments_, materials_);
  }
}

/**
 * @brief Load materials and set segments/materials on ray_cast_ if possible
 *
 * @param materials The material list
 */
void LightScanSim::MaterialsCallback(const light_scan_sim::msg::MaterialList::SharedPtr materials) {
  materials_ = *materials;
  materials_loaded_ = true;

  if (map_loaded_ && segments_loaded_ && materials_loaded_) {
    ray_cast_->SetSegments(segments_, materials_);
  }
}

/**
 * @brief Load segments and set segments/materials on ray_cast_ if possible
 *
 * @param segments The segment list
 */
void LightScanSim::SegmentsCallback(const light_scan_sim::msg::SegmentList::SharedPtr segments) {
  segments_ = *segments;
  segments_loaded_ = true;

  // Todo: Somehow use TF to transform segments into image space

  if (map_loaded_ && segments_loaded_ && materials_loaded_) {
    ray_cast_->SetSegments(segments_, materials_);
  }
}

/**
 * @brief Generate and publish the simulated laser scan
 */
void LightScanSim::Update() {
  if (!map_loaded_) {
    RCLCPP_WARN(get_logger(), "LightScanSim: Update called, no map yet");
    return;
  }

  // Broadcast the tf representing the map image
  geometry_msgs::msg::TransformStamped map_to_image_stamped;
  //map_to_image_stamped.transform = map_to_image_; // no = operator?
  map_to_image_stamped.transform = tf2::toMsg(map_to_image_);
  map_to_image_stamped.header.stamp = now();
  map_to_image_stamped.header.frame_id = map_.header.frame_id;
  map_to_image_stamped.child_frame_id = image_frame_;
  tf_broadcaster_.sendTransform(map_to_image_stamped);

  // Use that transform to generate a point in image space
  geometry_msgs::msg::TransformStamped image_to_laser;
  try{
    image_to_laser = tf_buffer_->lookupTransform(image_frame_, laser_frame_, rclcpp::Time(0), rclcpp::Duration(10.0));
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "LightScanSim: %s",ex.what());
    return;
  }

  // Convert that point from m to px
  cv::Point laser_point(image_to_laser.transform.translation.x/map_.info.resolution,
                        image_to_laser.transform.translation.y/map_.info.resolution);
  // And get the yaw
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion quat_msg = image_to_laser.transform.rotation;
  tf2::Quaternion quat_tf;
  tf2::convert(quat_msg, quat_tf);
  //quat_msg = tf2::toMsg(quat_tf);
  tf2::Matrix3x3 quat_matrix(quat_tf);
  quat_matrix.getRPY(roll, pitch, yaw);

  // Generate the ray cast laser scan at that point and orientation
  sensor_msgs::msg::LaserScan scan = ray_cast_->Scan(laser_point, yaw);

  // Set the header values
  scan.header.stamp = image_to_laser.header.stamp;  // Use correct time
  scan.header.frame_id = laser_frame_;  // set laser's tf

  // And publish the laser scan
  laser_pub_->publish(scan);
}

double LightScanSim::get_rate()
{
  return freq_hz_;
}
