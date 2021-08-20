#!/usr/bin/env python3

import rclpy
import message_filters as mf
import numpy as np
import cv2

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from carla_simulation.msg import LaneGeometry, LaneParameters

class OutputView(Node):

    def __init__(self):
        super().__init__('output_view')
        # Declare ROS parameters
        self.declare_parameters(namespace='',
                               	parameters=[('qos_length'),
                                            ('topic.rgb_image'),
                                            ('topic.lane_geometry'),
                                            ('topic.lane_parameters')])
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
        lane_geo_topic = self.get_parameter('topic.lane_geometry').get_parameter_value().string_value
        self.lane_geo_sub = mf.Subscriber(self,LaneGeometry,lane_geo_topic,qos_profile=qos_profile)
        lane_params_topic = self.get_parameter('topic.lane_parameters').get_parameter_value().string_value
        self.lane_params_sub = mf.Subscriber(self,LaneParameters,lane_params_topic,qos_profile=qos_profile)
        # Apply message filter
        self.timestamp_sync = mf.TimeSynchronizer([self.rgb_sub,self.lane_geo_sub,self.lane_params_sub],queue_size=qos_length)
        self.timestamp_sync.registerCallback(self.displayCallback)

    def nodeParams(self):
        self.do_once = True

    def displayCallback(self,rgb_msg,lane_geo_msg,lane_params_msg):
        rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg)
        if self.do_once:
            self.ploty = np.linspace(0,rgb_img.shape[0]-1,rgb_img.shape[0])
            self.minv = np.array(lane_geo_msg.transformation_matrix).reshape((3,3))
            self.do_once = False
        left_fit = lane_geo_msg.left_coefficients
        right_fit = lane_geo_msg.right_coefficients
        if left_fit == [0,0,0]:
            cv2.imshow(rgb_img)
            cv2.waitKey(1)
        else:
            left_fitx = left_fit[0] * self.ploty ** 2 + left_fit[1] * self.ploty + left_fit[2]
            right_fitx = right_fit[0] * self.ploty ** 2 + right_fit[1] * self.ploty + right_fit[2]
            color_warp = np.zeros_like(rgb_img).astype(np.uint8)
            pts_left = np.array([np.transpose(np.vstack([left_fitx,self.ploty]))])
            pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx,self.ploty])))])
            pts = np.hstack((pts_left, pts_right))
            cv2.fillPoly(color_warp,np.int32([pts]),(0,255,0))
            newwarp = cv2.warpPerspective(color_warp,self.minv,(rgb_img.shape[1],rgb_img.shape[0]))
            result = cv2.addWeighted(rgb_img,1,newwarp,0.3,0)
            cv2.putText(result,'Centre offset: ' + str(lane_params_msg.centre_offset) + ' m',(10,50),
                        cv2.FONT_HERSHEY_SIMPLEX,0.8,(235,52,189),2,cv2.LINE_AA)
            cv2.imshow(result)
            cv2.waitKey(1)