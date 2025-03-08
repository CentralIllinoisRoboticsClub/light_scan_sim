from launch import LaunchDescription
from launch_ros.actions import Node
from numpy import number

from launch.actions import ExecuteProcess

from ament_index_python.packages import get_package_share_directory

# https://github.com/ros-drivers/ros2_ouster_drivers/blob/eloquent-devel/ros2_ouster/launch/os1_launch.py

# https://github.com/stereolabs/zed-ros2-wrapper/blob/master/zed_wrapper/launch/zed.launch.py
# Also need to add config to the CMakeLists.txt install(DIRECTORY
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    params_yaml = get_package_share_directory('light_scan_sim') + "/config/config.yaml"
    
    scan_sim_node = Node(
        package="light_scan_sim",
        executable="light_scan_sim_node",
        name="light_scan_sim_node",
        parameters=[params_yaml],
    )
    
    ld.add_action(scan_sim_node)
    
    return ld
