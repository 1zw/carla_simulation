#!/usr/bin/env python3

import rclpy
import message_filters as mf
import numpy as np
import cv2

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ObjectDetection(Node):

    def __init__(self):
        super().__init__('object_detection')
        # Declare ROS parameters
        self.declare_parameters(namespace='',
                               	parameters=[('qos_length',0),
                                            ('topic.rgb_image',''),
                                            ('topic.depth_image',''),
                                            ('topic.segmentation','')])
        qos_length = self.get_parameter('qos_length').get_parameter_value().integer_value
        qos_profile = QoSProfile(depth=qos_length,
                                 history=QoSHistoryPolicy.KEEP_LAST,
                                 reliability=QoSReliabilityPolicy.RELIABLE)
        # Load cv_bridge
        self.bridge = CvBridge()
        # Create Subscribers
        rgb_topic = self.get_parameter('topic.rgb_image').get_parameter_value().string_value
        self.rgb_sub = mf.Subscriber(self,Image,rgb_topic,qos_profile=qos_profile)
        depth_topic = self.get_parameter('topic.depth_image').get_parameter_value().string_value
        self.depth_sub = mf.Subscriber(self,Image,depth_topic,qos_profile=qos_profile)
        segmentation_topic = self.get_parameter('topic.segmentation').get_parameter_value().string_value
        self.seg_sub = mf.Subscriber(self,Image,segmentation_topic,qos_profile=qos_profile)
        # Apply message filter
        self.timestamp_sync = mf.TimeSynchronizer([self.rgb_sub,self.depth_sub,self.seg_sub],queue_size=qos_length)
        self.timestamp_sync.registerCallback(self.imgCallback)

    def imgCallback(self,rgb_msg,depth_msg,seg_msg):


def main(args=None):
    rclpy.init(args=args)
    det_node = ObjectDetection()
    rclpy.spin(det_node)
    det_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    