/**
 * light_scan_sim light_scan_sim_node.cpp
 * @brief A ROS node that publishes laser scans
 *
 * @copyright 2017 Joseph Duchesne
 * @author Joseph Duchesne
 */

#include <rclcpp/rclcpp.hpp>
#include "light_scan_sim/light_scan_sim.h"

/**
 * @brief Init node and update the sim at the desired rate
 */
int main(int argc, char** argv){
  //Initiate ROS
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  auto lidar_sim_node = std::make_shared<LightScanSim>(node_options);

  RCLCPP_INFO(lidar_sim_node->get_logger(), "Starting Light Scan Simulator");
  int loop_hz = lidar_sim_node->get_rate();
  rclcpp::Rate rate(loop_hz);

  //rclcpp::spin(nav_states_node);
  while(rclcpp::ok())
  {
    rclcpp::spin_some(lidar_sim_node);
    lidar_sim_node->Update();
    rate.sleep();
  }

  return 0;
}
