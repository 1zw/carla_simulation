#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config_path = os.path.join(get_package_share_directory('carla_simulation'), 'config', 'params.yaml')  
    lane_node = Node(package = 'carla_simulation',
                     executable = 'lane_detection.py',
                     output = 'screen',
                     emulate_tty=True,
                     parameters = [config_path])
    det_node =  Node(package = 'carla_simulation',
                     executable = 'object_detection.py',
                     output = 'screen',
                     emulate_tty=True,
                     parameters = [config_path])
    ld.add_action(lane_node)
    ld.add_action(det_node)
    return ld