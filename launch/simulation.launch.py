#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config_path = os.path.join(get_package_share_directory('carla_simulation'), 'config', 'params.yaml')  
    base_node = Node(package = 'carla_simulation',
                     node_executable = 'lane_detection.py',
                     parameters = [config_path])
    ld.add_action(base_node)
    return ld