#!/usr/bin/env python3

import rclpy
import message_filters as mf
import numpy as np
import cv2

from rclpy.node import Node
from rclpy.qos import QoSProfile,QoSHistoryPolicy,QoSReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from carla_simulation.msg import Object,ObjectArray

class ObjectTracking(Node):

    def __init__(self):
        super().__init__('object_tracking')