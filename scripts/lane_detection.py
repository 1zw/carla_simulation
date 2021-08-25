#!/usr/bin/env python3

import rclpy
import message_filters as mf
import numpy as np
import cv2

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from carla_simulation.msg import LaneParameters

class LaneDetection(Node):

    def __init__(self):
        super().__init__('lane_detection')
        # Declare ROS parameters
        self.declare_parameters(namespace='',
                               	parameters=[('qos_length', 0),
                                            ('topic.rgb_image',''),
                                            ('topic.segmentation',''),
                                            ('topic.lane_parameters', ''),
                                            ('lane_color', [0,0,0]),
                                            ('roi.x1_offset', 0),
                                            ('roi.x2_offset', 0),
                                            ('roi.y_offset', 0),
                                            ('roi.width1', 0),
                                            ('roi.width2', 0),
                                            ('roi.height', 0),
                                            ('search_params.n_windows', 0),
                                            ('search_params.margin', 0),
                                            ('search_params.min_pixels', 0),
                                            ('search_params.mean_limit', [0,0]),
                                            ('m_per_pix.x', 0),
                                            ('m_per_pix.y', 0),
                                            ('frame_id.lane_parameters', ''),
                                            ('display', False)])
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
        segmentation_topic = self.get_parameter('topic.segmentation').get_parameter_value().string_value
        self.seg_sub = mf.Subscriber(self,Image,segmentation_topic,qos_profile=qos_profile)
        # Apply message filter
        self.timestamp_sync = mf.TimeSynchronizer([self.rgb_sub,self.seg_sub],queue_size=qos_length)
        self.timestamp_sync.registerCallback(self.imgCallback)
        # Create Publishers
        lane_params_topic = self.get_parameter('topic.lane_parameters').get_parameter_value().string_value
        self.lane_params_pub = self.create_publisher(LaneParameters,lane_params_topic,qos_profile)

    def nodeParams(self):
        self.lane_color = np.array(self.get_parameter('lane_color').get_parameter_value().integer_array_value, dtype=np.uint8)
        self.do_once = True
        self.sliding_window = True
        self.nwindows = self.get_parameter('search_params.n_windows').get_parameter_value().integer_value
        self.margin = self.get_parameter('search_params.margin').get_parameter_value().integer_value
        self.minpix = self.get_parameter('search_params.min_pixels').get_parameter_value().integer_value
        self.mean_limit = self.get_parameter('search_params.mean_limit').get_parameter_value().double_array_value
        self.xm_per_pix = self.get_parameter('m_per_pix.x').get_parameter_value().double_value
        self.ym_per_pix = self.get_parameter('m_per_pix.y').get_parameter_value().double_value
        self.lane_params_msg = LaneParameters()
        self.lane_params_msg.header.frame_id = self.get_parameter('frame_id.lane_parameters').get_parameter_value().string_value
        self.display = self.get_parameter('display').get_parameter_value().bool_value
        self.draw_area = True

    def imgCallback(self,rgb_msg,seg_msg):
        rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg)
        seg_img = self.bridge.imgmsg_to_cv2(seg_msg)
        if self.do_once:
            self.height = seg_img.shape[0]
            self.width = seg_img.shape[1]
            self.roiParams()
        binary = cv2.inRange(seg_img,self.lane_color,self.lane_color)
        binary = binary // 255
        binary = cv2.warpPerspective(binary,self.tm,binary.shape[::-1],flags=cv2.INTER_LINEAR)
        self.findLanes(binary)
        self.lane_params_msg.header.stamp = seg_msg.header.stamp
        self.lane_params_pub.publish(self.lane_params_msg)
        if self.display:
            self.viewOutput(rgb_img)

    def roiParams(self):
        roi_y = self.get_parameter('roi.y_offset').get_parameter_value().integer_value
        roi_x1 = self.get_parameter('roi.x1_offset').get_parameter_value().integer_value
        roi_x2 = self.get_parameter('roi.x2_offset').get_parameter_value().integer_value
        roi_h = self.get_parameter('roi.height').get_parameter_value().integer_value
        roi_w1 = self.get_parameter('roi.width1').get_parameter_value().integer_value
        roi_w2 = self.get_parameter('roi.width2').get_parameter_value().integer_value
        src = np.float32([[(roi_x1 + roi_w1),roi_y],
                          [(roi_x2 + roi_w2),(roi_y + roi_h)],
                          [roi_x2,(roi_y + roi_h)],
                          [roi_x1,roi_y]])
        dst = np.float32([[3 * self.width // 4,0],[3 * self.width // 4,self.height],[self.width // 4,self.height],[self.width // 4,0]])
        self.tm = cv2.getPerspectiveTransform(src,dst)
        self.tminv = cv2.getPerspectiveTransform(dst,src)
        self.ploty = np.linspace(0,self.height-1,self.height)
        self.do_once = False

    def findLanes(self,img):
        nonzero = img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        left_lane_inds = []
        right_lane_inds = []
        histogram = np.sum(img[self.height // 2:,:], axis=0)
        if self.mean_limit[0] < histogram.mean() < self.mean_limit[1]:
            if self.sliding_window: # Searching by sliding window
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
                    good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                                        (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
                    good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                                        (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
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
        if len(leftx) > 0 and len(rightx) > 0:
            self.left_fit = np.polyfit(lefty, leftx, 2)
            self.right_fit = np.polyfit(righty, rightx, 2)
            # Calculate lane radius
            y_eval = self.height * self.ym_per_pix
            self.lane_params_msg.left_radius = ((1 + (2 * self.left_fit[0] * y_eval + self.left_fit[1]) ** 2) ** 1.5) / np.abs(2 * self.left_fit[0])
            self.lane_params_msg.right_radius = ((1 + (2 * self.right_fit[0] * y_eval + self.right_fit[1]) ** 2) ** 1.5) / np.abs(2 * self.right_fit[0])
            self.lane_params_msg.centre_radius = (self.lane_params_msg.left_radius + self.lane_params_msg.right_radius) / 2
            # Calculate centre offset
            bottom_left = self.left_fit[0] * (self.height ** 2) + self.left_fit[1] * self.height + self.left_fit[2]
            bottom_right = self.right_fit[0] * (self.height ** 2) + self.right_fit[1] * self.height + self.right_fit[2]
            center_lane = (bottom_right - bottom_left) / 2 + bottom_left
            self.lane_params_msg.centre_offset = (np.abs(self.width / 2) - np.abs(center_lane)) * self.xm_per_pix
        else:
            self.sliding_window = True
            self.draw_area = False
            self.lane_params_msg.left_radius = 0.0
            self.lane_params_msg.right_radius = 0.0
            self.lane_params_msg.centre_offset = 0.0

    def viewOutput(self,img):
        if self.draw_area:
            left_fitx = self.left_fit[0] * self.ploty ** 2 + self.left_fit[1] * self.ploty + self.left_fit[2]
            right_fitx = self.right_fit[0] * self.ploty ** 2 + self.right_fit[1] * self.ploty + self.right_fit[2]
            color_warp = np.zeros_like(img).astype(np.uint8)
            pts_left = np.array([np.transpose(np.vstack([left_fitx,self.ploty]))])
            pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx,self.ploty])))])
            pts = np.hstack((pts_left, pts_right))
            cv2.fillPoly(color_warp,np.int32([pts]),(0,255,0))
            color_warp = cv2.warpPerspective(color_warp,self.tminv,(img.shape[1],img.shape[0]))
            img = cv2.addWeighted(img,1,color_warp,0.3,0)
            cv2.putText(img,'Centre offset: ' + str(round(self.lane_params_msg.centre_offset,4)) + ' m',(10,50),
                        cv2.FONT_HERSHEY_SIMPLEX,0.5,(245,130,65),2,cv2.LINE_AA)
        cv2.imshow('image',img)
        cv2.waitKey(1)
        self.draw_area = True
        

def main(args=None):
    rclpy.init(args=args)
    det_node = LaneDetection()
    rclpy.spin(det_node)
    det_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
