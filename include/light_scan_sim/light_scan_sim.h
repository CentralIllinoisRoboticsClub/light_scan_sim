/**
 * light_scan_sim light_scan_sim.h
 * @brief Monitor map and tf data, publish simulated laser scan
 *
 * @copyright 2017 Joseph Duchesne
 * @author Joseph Duchesne
 * 
 */

#ifndef LIGHT_SCAN_SIM_LIGHT_SCAN_SIM_H
#define LIGHT_SCAN_SIM_LIGHT_SCAN_SIM_H

#include <stdint.h>

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <light_scan_sim/msg/material_list.hpp>
#include <light_scan_sim/msg/segment_list.hpp>

#include "light_scan_sim/ray_cast.h"

class LightScanSim: public rclcpp::Node
{
  double freq_hz_;

  // Internal data
  nav_msgs::msg::OccupancyGrid map_;
  light_scan_sim::msg::MaterialList materials_;
  light_scan_sim::msg::SegmentList segments_;
  tf2::Transform map_to_image_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  bool map_loaded_ = false;
  bool segments_loaded_ = false;
  bool materials_loaded_ = false;

  // Publishers and subscribers
  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::OccupancyGrid> > map_sub_;
  std::shared_ptr<rclcpp::Subscription<light_scan_sim::msg::MaterialList> > materials_sub_;
  std::shared_ptr<rclcpp::Subscription<light_scan_sim::msg::SegmentList> > segments_sub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan> > laser_pub_;
  std::shared_ptr<RayCast> ray_cast_;

  // Configurable options
  std::string map_topic_ = "/map";
  std::string materials_topic_ = "/map_materials";
  std::string segments_topic_ = "/map_segments";
  std::string laser_topic_ = "/scan";

  std::string image_frame_ = "/map_image";
  std::string laser_frame_ = "/initialpose";

  public:
    LightScanSim(const rclcpp::NodeOptions& node_options);

    void MapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr grid);
    void MaterialsCallback(const light_scan_sim::msg::MaterialList::SharedPtr materials);
    void SegmentsCallback(const light_scan_sim::msg::SegmentList::SharedPtr segments);

    void Update();
    double get_rate();

};

#endif
