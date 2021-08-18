#!/usr/bin/env python3

import rclpy
import numpy as np
import cv2

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from carla_simulation.msg import LaneGeometry
from carla_simulation.msg import LaneParameters

class LaneDetection(Node):

	def __init__(self):
        super().__init__('lane_detection')
        # Declare ROS parameters
        self.declare_parameters(namespace='',
                               	parameters=[('qos_length'),
                                            ('topic.segmentation'),
                                            ('topic.lane_geometry'),
                                            ('topic.lane_parameters'),
                                            ('lane_color'),
                                            ('roi.x1_offset'),
                                            ('roi.x2_offset'),
                                            ('roi.y_offset'),
                                            ('roi.width1'),
                                            ('roi.width2'),
                                            ('roi.height'),
                                            ('debug_view'),
                                            ('search_params.n_windows'),
                                            ('search_params.margin'),
                                            ('search_params.min_pixels'),
                                            ('search_params.mean_limit'),
                                            ('frame_id.lane_geometry'),
                                            ('m_per_pix.x'),
                                            ('m_per_pix.y'),
                                            ('frame_id.lane_parameters')])
        self.nodeParams()
        qos_length = self.get_parameter('qos_length').get_parameter_value().integer_value
        qos_profile = QoSProfile(depth=qos_length,
                                 history=QoSHistoryPolicy.KEEP_LAST,
                                 reliability=QoSReliabilityPolicy.RELIABLE)
        # Load cv_bridge
        self.bridge = CvBridge()
        # Create Subscribers
        segmentation_topic = self.get_parameter('topic.segmentation').get_parameter_value().string_value
        self.seg_sub = self.create_subscription(Image,segmentation_topic,self.segCallback,qos_profile)
        # Create Publishers
        lane_geo_topic = self.get_parameter('topic.lane_geometry').get_parameter_value().string_value
        self.lane_geo_pub = self.create_publisher(LaneGeometry,lane_geo_topic,qos_profile)
        lane_params_topic = self.get_parameter('topic.lane_parameters').get_parameter_value().string_value
        self.lane_params_pub = self.create_publisher(LaneParameters,lane_params_topic,qos_profile)

    def nodeParams(self):
        self.lane_color = np.array(self.get_parameter('lane_color').get_parameter_value().integer_array_value)
        self.do_once = True
        self.debug_view = self.get_parameter('debug_view').get_parameter_value().bool_value
        self.sliding_window = True
        self.nwindows = self.get_parameter('search_params.n_windows').get_parameter_value().integer_value
        self.margin = self.get_parameter('search_params.margin').get_parameter_value().integer_value
        self.min_pixels = self.get_parameter('search_params.min_pixels').get_parameter_value().integer_value
        self.mean_limit = self.get_parameter('search_params.mean_limit').get_parameter_value().double_value
        self.lane_geo_msg = LaneGeometry()
        self.lane_geo_msg.header.frame_id = self.get_parameter('frame_id.lane_geometry').get_parameter_value().string_value
        self.xm_per_pix = self.get_parameter('m_per_pix.x').get_parameter_value().double_value # 3.7 meters / 360 pixels
        self.ym_per_pix = self.get_parameter('m_per_pix.y').get_parameter_value().double_value # 16 meters / 480 pixels
        self.lane_params_msg = LaneParameters()
        self.lane_params_msg.header.frame_id = self.get_parameter('frame_id.lane_parameters').get_parameter_value().string_value

    def segCallback(self,msg):
    	seg_img = self.bridge.imgmsg_to_cv2(msg)
        binary = cv2.inRange(seg_img, self.lane_color, self.lane_color)
        binary = self.perspectiveTransform(binary)
        if self.debug_view == True:
            image = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
            image = cv2.rectangle(binary,((self.width // 4),0),((3 * self.width // 4),self.height), (255,0,0), 2)
            cv2.imshow(image)
            cv2.waitKey(1)
        self.findLanes(binary)
        self.publishLaneMsgs(self,msg.header.stamp)

    def perspectiveTransform(self,img):
        if self.do_once:
            self.height = img.shape[0]
            self.width = img.shape[1]
            roi_x1 = self.get_parameter('roi.x1_offset').get_parameter_value().integer_value
            roi_x2 = self.get_parameter('roi.x2_offset').get_parameter_value().integer_value
            roi_y = self.get_parameter('roi.y_offset').get_parameter_value().integer_value
            roi_h = self.get_parameter('roi.height').get_parameter_value().integer_value
            roi_w1 = self.get_parameter('roi.width1').get_parameter_value().integer_value
            roi_w2 = self.get_parameter('roi.width2').get_parameter_value().integer_value
            src = np.float32([[(roi_x1+roi_w1),roi_y],
                              [(roi_x2+roi_w2),(roi_y+roi_h)],
                              [roi_x2,(roi_y+roi_h)],
                              [roi_x1,roi_y]])
            dst = np.float32([[3 * self.width // 4,0],[3 * self.width // 4,self.height],[self.width // 4,self.height],[self.width // 4,0]])
            self.M = cv2.getPerspectiveTransform(src,dst)
            Minv = cv2.getPerspectiveTransform(dst,src)
            self.lane_geo_msg.transformation_matrix = Minv.flatten().tolist()
            self.do_once = False
        return cv2.warpPerspective(img,self.M,img.shape[::-1],flags=cv2.INTER_LINEAR)

    def findLanes(self,img):
        nonzero = img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        if self.sliding_window: # Searching by sliding window
            left_lane_inds = []
            right_lane_inds = []
            histogram = np.sum(img[self.height // 2:,:], axis=0)
            if np.mean(histogram) < self.mean_limit:
                leftx_current = np.argmax(histogram[:self.width // 2])
                rightx_current = np.argmax(histogram[self.width // 2:]) + self.width // 2
                window_height = self.height // self.nwindows
                for window in range(self.nwindows):
                    win_y_low = self.height - (window + 1) * window_height
                    win_y_high = self.height - window * window_height
                    win_xleft_low = leftx_current - self.margin
                    win_xleft_high = leftx_current + self.margin
                    win_xright_low = rightx_current - self.margin
                    win_xright_high = rightx_current + self.margin
                    good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
                    good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
                    left_lane_inds.append(good_left_inds)
                    right_lane_inds.append(good_right_inds)
                    if len(good_left_inds) > self.minpix:
                        leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
                    if len(good_right_inds) > self.minpix:        
                        rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
                try:
                    left_lane_inds = np.concatenate(left_lane_inds)
                    right_lane_inds = np.concatenate(right_lane_inds)
                except ValueError:
                    pass
                self.sliding_window = False
        else: # Searching around polynomial
            left_lane_inds = ((nonzerox > (self.left_fit[0] * (nonzeroy ** 2) + self.left_fit[1] * nonzeroy + self.left_fit[2] - self.margin)) &
                             (nonzerox < (self.left_fit[0] * (nonzeroy ** 2) + self.left_fit[1] * nonzeroy + self.left_fit[2] + self.margin)))
            right_lane_inds = ((nonzerox > (self.right_fit[0] * (nonzeroy ** 2) + self.right_fit[1] * nonzeroy + self.right_fit[2] - self.margin)) &
                              (nonzerox < (self.right_fit[0] * (nonzeroy ** 2) + self.right_fit[1] * nonzeroy + self.right_fit[2] + self.margin)))
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        if len(leftx) > 0 and len(rightx > 0):
            self.left_fit = np.polyfit(lefty, leftx, 2)
            self.right_fit = np.polyfit(righty, rightx, 2)
            self.lane_geo_msg.left_coefficients = left_fit.tolist()
            self.lane_geo_msg.left_coefficients = right_fit.tolist()
            # Calculate lane radius
            y_eval = self.height * self.ym_per_pix
            self.lane_params_msg.left_radius = ((1 + (2 * self.left_fit[0] * y_eval + self.left_fit[1]) ** 2) ** 1.5) / np.absolute(2 * self.left_fit[0])
            self.lane_params_msg.right_radius = ((1 + (2 * self.right_fit[0] * y_eval + self.right_fit[1]) ** 2) ** 1.5) / np.absolute(2 * self.right_fit[0])
            # Calculate centre offset
            bottom_left = self.left_fit[0] * (self.height ** 2) + self.left_fit[1] * self.height + self.left_fit[2]
            bottom_right = self.right_fit[0] * (self.height ** 2) + self.right_fit[1] * self.height + self.right_fit[2]
            center_lane = (bottom_right - bottom_left) / 2 + bottom_left
            self.lane_params_msg.center_offset = (np.abs(self.width / 2) - np.abs(center_lane)) * self.xm_per_pix
        else: # TODO
            self.sliding_window = True
            self.lane_geo_msg.left_coefficients = []
            self.lane_geo_msg.left_coefficients = []
            self.lane_params_msg.left_radius = 0
            self.lane_params_msg.right_radius = 0
            self.lane_params_msg.center_offset = 0

    def publishLaneMsgs(self,stamp):
        self.lane_geo_msg.header.stamp = stamp
        self.lane_params_msg.header.stamp = stamp
        self.lane_geo_pub.publish(self.lane_geo_msg)
        self.lane_params_pub.publish(self.lane_params_msg)


def main(args=None):
    rclpy.init(args=args)
    det_node = LaneDetection()
    rclpy.spin(det_node)
    det_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
