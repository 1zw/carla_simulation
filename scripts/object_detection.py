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

class ObjectDetection(Node):

    def __init__(self):
        super().__init__('object_detection')
        # Declare ROS parameters
        self.declare_parameters(namespace='',
                               	parameters=[('qos_length',0),
                                            ('topic.depth_image',''),
                                            ('topic.segmentation',''),
                                            ('topic.objects',''),
                                            ('obj_class.name',[]),
                                            ('obj_class.color.person',[]),
                                            ('obj_class.color.vehicle',[]),
                                            ('frame_id.objects','')])
        self.nodeParams()
        qos_length = self.get_parameter('qos_length').get_parameter_value().integer_value
        qos_profile = QoSProfile(depth=qos_length,
                                 history=QoSHistoryPolicy.KEEP_LAST,
                                 reliability=QoSReliabilityPolicy.RELIABLE)
        # Load cv_bridge
        self.bridge = CvBridge()
        # Create Subscribers
        depth_topic = self.get_parameter('topic.depth_image').get_parameter_value().string_value
        self.depth_sub = mf.Subscriber(self,Image,depth_topic,qos_profile=qos_profile)
        segmentation_topic = self.get_parameter('topic.segmentation').get_parameter_value().string_value
        self.seg_sub = mf.Subscriber(self,Image,segmentation_topic,qos_profile=qos_profile)
        # Apply message filter
        self.timestamp_sync = mf.TimeSynchronizer([self.depth_sub,self.seg_sub],queue_size=qos_length)
        self.timestamp_sync.registerCallback(self.imgCallback)
        # Create Publishers
        obj_topic = self.get_parameter('topic.objects').get_parameter_value().string_value
        self.obj_pub = self.create_publisher(ObjectArray,obj_topic,qos_profile)

    def nodeParams(self):
        class_name = self.get_parameter('obj_class.name').get_parameter_value().string_array_value
        self.class_color = {}
        for name in class_name:
            self.class_color[name] = np.array(self.get_parameter('obj_class.color.' + name).get_parameter_value().integer_array_value,dtype=np.uint8)

    def imgCallback(self,depth_msg,seg_msg):
        self.obj_msg = ObjectArray()
        self.obj_msg.header.frame_id = self.get_parameter('frame_id.lane_parameters').get_parameter_value().string_value
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg)
        seg_img = self.bridge.imgmsg_to_cv2(seg_msg)
        for name in self.class_color:
            binary = self.seg2Binary(seg_img,self.class_color[name])
            self.imgClassification(binary,depth_img,name)
        self.obj_msg.header.stamp = seg_msg.header.stamp
        self.obj_pub.publish(self.lane_params_msg)

    def seg2Binary(self,img,color):
        img = cv2.inRange(img,color,color)
        img = img // 255
        return img

    def imgClassification(self,b_img,d_img,name):
        contours,hierarchy = cv2.findContours(b_img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for i,h in enumerate(hierarchy[0]):
            if h[-1] == -1:
                x,y,w,h = cv2.boundingRect(contours[i])
                img_roi = d_img[y:y + h,x:x + w]
                z_real = np.median(img_roi)
                temp_msg = Object()
                if name == 'person':
                    temp_msg.classification = temp_obj.CLASSIFICATION_PERSON
                elif name == 'vehicle':
                    temp_msg.classification = temp_obj.CLASSIFICATION_VEHICLE
                temp_msg.detection_box = [x,y,x + w,y + h]
                temp_msg.depth = z_real
                self.obj_msg.objects.append(temp_msg)          


def main(args=None):
    rclpy.init(args=args)
    det_node = ObjectDetection()
    rclpy.spin(det_node)
    det_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    