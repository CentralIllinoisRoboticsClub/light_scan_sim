/**
 * light_scan_sim wall_segment_sim.h
 * @brief Simulate laser rays against wall segments
 *
 * @copyright 2017 Joseph Duchesne
 * @author Joseph Duchesne
 */

#ifndef LIGHT_SCAN_SIM_WALL_SEGMENT_SIM_H
#define LIGHT_SCAN_SIM_WALL_SEGMENT_SIM_H

#include <rclcpp/rclcpp.hpp>
#include <opencv2/core/core.hpp>
#include <light_scan_sim/msg/segment_list.hpp>
#include <light_scan_sim/msg/material_list.hpp>
#include <Box2D/Box2D.h>
#include <random>

class WallSegmentSim {
  private:
    light_scan_sim::msg::SegmentList segments_;
    light_scan_sim::msg::MaterialList materials_;

    std::shared_ptr<b2World> world_ = nullptr;
  
    void InitializeWorld();

  public:
    WallSegmentSim(light_scan_sim::msg::SegmentList segments, light_scan_sim::msg::MaterialList materials);

    bool Trace(double x, double y, double theta, double length, double ray_max, double &range);
  
};

#endif

