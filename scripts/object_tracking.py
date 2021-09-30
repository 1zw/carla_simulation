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
from carla_simulation.sort import *

class ObjectTracking(Node):

    def __init__(self):
        super().__init__('object_tracking')
        # Declare ROS parameters
        self.declare_parameters(namespace='',
                               	parameters=[('qos_length',0)])
        self.nodeParams()
        qos_length = self.get_parameter('qos_length').get_parameter_value().integer_value
        qos_profile = QoSProfile(depth=qos_length,
                                 history=QoSHistoryPolicy.KEEP_LAST,
                                 reliability=QoSReliabilityPolicy.RELIABLE)
        # Load cv_bridge
        self.bridge = CvBridge()
        # Create instance of SORT
        self.mot_tracker = Sort()
        # Create Subscribers
        obj_topic = self.get_parameter('topic.objects').get_parameter_value().string_value
        self.obj_sub = mf.Subscriber(self,ObjectArray,obj_topic,qos_profile=qos_profile)
        rgb_topic = self.get_parameter('topic.rgb_image').get_parameter_value().string_value
        self.rgb_sub = mf.Subscriber(self,Image,rgb_topic,qos_profile=qos_profile)
        # Apply message filter
        self.timestamp_sync = mf.TimeSynchronizer([self.obj_sub,self.rgb_sub],queue_size=qos_length)
        self.timestamp_sync.registerCallback(self.objCallback)

    def nodeParams(self):
        pass

    def objCallback(self,obj_msg,rgb_msg):
        self.obj_track_msg = ObjectArray()
        detections = np.empty((0, 5))
        for obj in obj_msg.objects:
            x1,y1,x2,y2 = obj.detection_box
            temp = np.array([[x1,y1,x2,y2,obj.classification]])
            detections = np.append(detections,temp,axis=0)
        track_bbs_ids = self.mot_tracker.update(detections)
        for x1,y1,x2,y2,obj_id,obj_class in track_bbs_ids:
            temp_msg = Object()
            temp_msg.id = int(obj_id)
            temp_msg.classification = int(obj_class)
            temp_msg.detection_box = [x1,y1,x2,y2]
            self.obj_track_msg.objects.append(temp_msg)


def main(args=None):
    rclpy.init(args=args)
    track_node = ObjectTracking()
    rclpy.spin(track_node)
    track_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
  