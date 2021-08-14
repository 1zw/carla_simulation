#!/usr/bin/env python3

import rclpy
import numpy as np
import cv2

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, SetParametersResult
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
                                            ('segmentation_topic','',self.addDescription()),
                                            ('lane_geometry_topic','',self.addDescription()),
                                            ('lane_parameters_topic','',self.addDescription()),
                                            ('lane_color',[0,0,0]),
                                            ('debug_view',False,self.addDescription()),
                                            ('lane_geometry_frame_id','',self.addDescription()),
                                            ('xm_per_pix'),
                                            ('ym_per_pix'),
                                            ('lane_parameters_frame_id','',self.addDescription())])
        self.nodeParams()

        self.add_on_set_parameters_callback(self.paramsCallback)

        qos_length = self.get_parameter('qos_length').get_parameter_value().integer_value
        qos_profile = QoSProfile(depth=qos_length,
                                 history=QoSHistoryPolicy.KEEP_LAST,
                                 reliability=QoSReliabilityPolicy.RELIABLE)

        # Load cv_bridge
        self.bridge = CvBridge()

        # Create Subscribers
        self.seg_sub = self.create_subscription(Image,self.segmentation_topic,self.segCallback,qos_profile)

        # Create Publishers
        self.lane_geo_pub = self.create_publisher(LaneGeometry,self.lane_geo_topic,qos_profile)
        self.lane_params_pub = self.create_publisher(LaneParameters,self.lane_params_topic,qos_profile)

    def addDescription(self,from_value=None,to_value=None,step=None):
        descriptor = ParameterDescriptor()
        if None not in [from_value,to_value,step]:
            integer_range = IntegerRange()
            integer_range.from_value = from_value
            integer_range.to_value = to_value
            integer_range.step = step
            descriptor.integer_range = [integer_range]
        descriptor.description = "-"
        return descriptor

    def nodeParams(self):
        self.segmentation_topic = self.get_parameter('segmentation_topic').get_parameter_value().string_value
        self.lane_geo_topic = self.get_parameter('lane_geometry_topic').get_parameter_value().string_value
        self.lane_params_topic = self.get_parameter('lane_parameters_topic').get_parameter_value().string_value
        self.lane_color = np.array(self.get_parameter('lane_color').get_parameter_value().integer_array_value)
        self.do_on_set = True
        self.declared = False
        self.roi_vars = ['roi/x1_offset','roi/x2_offset','roi/y_offset','roi/width1','roi/width2','roi/height']
        self.debug_view = self.get_parameter('debug_view').get_parameter_value().bool_value
        self.lane_geo_msg = LaneGeometry()
        self.lane_geo_msg.header.frame_id = self.get_parameter('lane_geometry_frame_id').get_parameter_value().string_value
        self.xm_per_pix = self.get_parameter('xm_per_pix').get_parameter_value().double_value
        self.ym_per_pix = self.get_parameter('ym_per_pix').get_parameter_value().double_value
        self.lane_params_msg = LaneParameters()
        self.lane_params_msg.header.frame_id = self.get_parameter('lane_parameters_frame_id').get_parameter_value().string_value

    def paramsCallback(self,params):
        success = False
    	for param in params:
            if param.type_ == Parameter.Type.INTEGER:
                if param.name in self.roi_vars:
                    self.do_on_set = True
                    success = True
            elif param.type_ == Parameter.Type.BOOL:
                if param.name == 'debug_view':
                    self.debug_view = param.value
                    success = True
	    return SetParametersResult(successful=success)

    def segCallback(self,msg):
    	seg_img = self.bridge.imgmsg_to_cv2(msg)
        binary = cv2.inRange(seg_img, self.lane_color, self.lane_color)
        binary = self.perspectiveTransform(binary)
        
        if self.debug_view == True:
            image = cv2.rectangle(binary,((self.width//4),0),((3*self.width//4),self.height), (255,0,0), 2)
            cv2.imshow(image)
            cv2.waitKey(1)
        
        self.findLanes(binary)

        self.publishLaneMsgs(self,msg.header.stamp)

    def perspectiveTransform(self,img):
        if self.do_on_set == True:
            self.height = img.shape[0]
            self.width = img.shape[1]

            if self.declared == False:
                self.declare_parameters(namespace='',
                                        parameters=[('roi/x1_offset',0,self.addDescription(0,self.width,1)),
                                                    ('roi/x2_offset',0,self.addDescription(0,self.width,1)),
                                                    ('roi/y_offset',0,self.addDescription(0,self.height,1)),
                                                    ('roi/width1',0,self.addDescription(0,self.width,1)),
                                                    ('roi/width2',0,self.addDescription(0,self.width,1)),
                                                    ('roi/height',0,self.addDescription(0,self.height,1))])
                self.declared = True

            roi_x1 = self.get_parameter('roi/x1_offset').get_parameter_value().integer_value
            roi_x2 = self.get_parameter('roi/x2_offset').get_parameter_value().integer_value
            roi_y = self.get_parameter('roi/y_offset').get_parameter_value().integer_value
            roi_h = self.get_parameter('roi/height').get_parameter_value().integer_value
            roi_w1 = self.get_parameter('roi/width1').get_parameter_value().integer_value
            roi_w2 = self.get_parameter('roi/width2').get_parameter_value().integer_value

            src = np.float32([[(roi_x1+roi_w1),roi_y],
                              [(roi_x2+roi_w2),(roi_y+roi_h)],
                              [roi_x2,(roi_y+roi_h)],
                              [roi_x1,roi_y]])
            dst = np.float32([[self.width,0],[self.width,self.height],[0,self.height],[0,0]])
            self.M = cv2.getPerspectiveTransform(src,dst)
            Minv = cv2.getPerspectiveTransform(dst,src)
            self.lane_geo_msg.transformation_matrix = Minv.flatten().tolist()
            self.do_on_set = False

        return cv2.warpPerspective(img,self.M,img.shape[::-1],flags=cv2.INTER_LINEAR)

    def findLanes(self,img):
        nonzero = img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        left_lane_inds = ((nonzeroy >= 0) & (nonzeroy < self.height) & (nonzerox >= 0) & (nonzerox < self.width//2)).nonzero()[0]
        right_lane_inds = ((nonzeroy >= 0) & (nonzeroy < self.height) & (nonzerox >= self.width//2) & (nonzerox < self.width)).nonzero()[0]
        
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)
        self.lane_geo_msg.left_coefficients = left_fit.tolist()
        self.lane_geo_msg.left_coefficients = right_fit.tolist()

        # Calculate lane radius
        y_eval = self.height * self.ym_per_pix
        self.lane_params_msg.left_radius = ((1 + (2 * left_fit[0] * y_eval + left_fit[1]) ** 2) ** 1.5) / np.absolute(2 * left_fit[0])
        self.lane_params_msg.right_radius = ((1 + (2 * right_fit[0] * y_eval + right_fit[1]) ** 2) ** 1.5) / np.absolute(2 * right_fit[0])

        # Calculate centre offset
        bottom_left = left_fit[0] * (self.height ** 2) + left_fit[1] * self.height + left_fit[2]
        bottom_right = right_fit[0] * (self.height ** 2) + right_fit[1] * self.height + right_fit[2]
        center_lane = (bottom_right - bottom_left) / 2 + bottom_left
        self.lane_params_msg.center_offset = (np.abs(self.width / 2) - np.abs(center_lane)) * self.xm_per_pix

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
