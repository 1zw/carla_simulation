#!/usr/bin/env python3

import rclpy
import message_filters as mf
import math
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
                                            ('topic.segmentation',''),
                                            ('color.person',[0,0,0]),
                                            ('color.vehicle'[0,0,0]),
                                            ('fov_per_pix',0)])
        self.nodeParams()
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

    def nodeParams(self):
        person_color = np.array(self.get_parameter('color.person').get_parameter_value().integer_array_value, dtype=np.uint8)
        vehicle_color = np.array(self.get_parameter('color.vehicle').get_parameter_value().integer_array_value, dtype=np.uint8)
        self.objects = {'person':person_color, 'vehicle':vehicle_color}
        self.display = self.get_parameter('display').get_parameter_value().bool_value
        self.fov_per_pix = self.get_parameter('fov_per_pix').get_parameter_value().double_value

    def imgCallback(self,rgb_msg,depth_msg,seg_msg):
        self.num_detections = 0
        self.detection_classes = []
        self.detection_boxes = []
        self.detection_points = []
        rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg)
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg)
        seg_img = self.bridge.imgmsg_to_cv2(seg_msg)
        for obj in self.objects:
            binary = self.seg2Binary(seg_img,self.objects[obj])
            self.findContours(binary,obj)
        self.objectLocalization(depth_img)
        if self.display:
            self.viewDetections(rgb_img)

    def seg2Binary(self,img,object_color):
        img = cv2.inRange(img,object_color,object_color)
        img = img // 255
        return img

    def findContours(self,img,object_class):
        contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for i,h in enumerate(hierarchy[0]):
            if h[-1] == -1:
                self.num_detections += 1
                self.detection_classes.append(object_class)
                x,y,w,h = cv2.boundingRect(contours[i])
                self.detection_boxes.append([x,y,w,h])
    
    def viewDetections(self,img):
        for i,obj_class in enumerate(self.detection_classes):
            color = self.objects[obj_class].tolist()
            x,y,w,h = self.detection_boxes[i]
            cv2.rectangle(img,(x,y),(x + w,y + h),color,1)
            cv2.putText(img,obj_class,(x,y - 2),cv2.FONT_HERSHEY_SIMPLEX,0.3,color,1)
        cv2.imshow('detections',img)
        cv2.waitKey(1)

    def objectLocalization(self,img):
        for x,y,w,h in self.detection_boxes:
            img_roi = img[y:y + h, x:x + w]
            z_real = np.median(img_roi)
            x_real = z_real * math.tan((img.shape[1] // 2 - x - w // 2) * self.fov_per_pix)
            self.detection_points.append([x_real,z_real])


def main(args=None):
    rclpy.init(args=args)
    det_node = ObjectDetection()
    rclpy.spin(det_node)
    det_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    