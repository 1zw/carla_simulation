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
                               	parameters=[('qos_length',0),
                                            ('topic.untracked_obj',''),
                                            ('topic.rgb_image',''),
                                            ('topic.tracked_obj',''),
                                            ('obj_class.id',[]),
                                            ('obj_class.name',[]),
                                            ('display',False),
                                            ('frame_id.tracked_obj','')])
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
        obj_topic = self.get_parameter('topic.untracked_obj').get_parameter_value().string_value
        self.obj_sub = mf.Subscriber(self,ObjectArray,obj_topic,qos_profile=qos_profile)
        rgb_topic = self.get_parameter('topic.rgb_image').get_parameter_value().string_value
        self.rgb_sub = mf.Subscriber(self,Image,rgb_topic,qos_profile=qos_profile)
        # Apply message filter
        self.timestamp_sync = mf.TimeSynchronizer([self.obj_sub,self.rgb_sub],queue_size=qos_length)
        self.timestamp_sync.registerCallback(self.objCallback)
        # Create Publishers
        obj_topic = self.get_parameter('topic.tracked_obj').get_parameter_value().string_value
        self.obj_pub = self.create_publisher(ObjectArray,obj_topic,qos_profile)

    def nodeParams(self):
        self.display = self.get_parameter('display').get_parameter_value().bool_value
        class_id = self.get_parameter('obj_class.id').get_parameter_value().integer_array_value
        class_name = self.get_parameter('obj_class.name').get_parameter_value().integer_array_value
        self.class_dict = {}
        for i,id_ in enumerate(class_id):
            self.class_dict[int(id_)] = class_name[i]

    def objCallback(self,obj_msg,rgb_msg):
        self.obj_track_msg = ObjectArray()
        self.obj_track_msg.header.frame_id = self.get_parameter('frame_id.tracked_obj').get_parameter_value().string_value
        detections = np.empty((0, 6))
        for obj in obj_msg.objects:
            x1,y1,x2,y2 = obj.detection_box
            temp = np.array([[x1,y1,x2,y2,obj.classification,obj.depth]])
            detections = np.append(detections,temp,axis=0)
        track_bbs_ids = self.mot_tracker.update(detections)
        for x1,y1,x2,y2,obj_id,obj_class,obj_depth in track_bbs_ids:
            temp_msg = Object()
            temp_msg.id = int(obj_id)
            temp_msg.classification = int(obj_class)
            temp_msg.detection_box = [x1,y1,x2,y2]
            temp_msg.depth = obj_depth
            self.obj_track_msg.objects.append(temp_msg)
        self.obj_track_msg.header.stamp = obj_msg.header.stamp
        self.obj_pub.publish(self.obj_track_msg)
        if self.display:
            viewTrackedObjects(rgb_msg)

    def viewTrackedObjects(self,img_msg):
        img = self.bridge.imgmsg_to_cv2(img_msg)
        for obj in obj_track_msg.objects:
            x1,y1,x2,y2 = obj.detection_box
            x1,y1,x2,y2 = int(x1),int(y1),int(x2),int(y2)
            text_ = self.class_dict[obj.classification] + "-" + str(obj.id)
            cv2.rectangle(img,(x1,y1),(x2,y2),(255,255,0),1)
            cv2.rectangle(img,(x1,y1 - 15),(x1 + len(text_) * 9, y1),(255,255,0),-1)
            cv2.putText(img,text_,(x1,y1 - 2),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),1)
        cv2.imshow('image',img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    track_node = ObjectTracking()
    rclpy.spin(track_node)
    track_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
