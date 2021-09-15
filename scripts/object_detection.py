#!/usr/bin/env python3

import rclpy
import message_filters as mf
import os
import pathlib
import numpy as np
import cv2
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge

from collections import defaultdict
from io import StringIO

from object_detection.utils import ops as utils_ops
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

utils_ops.tf = tf.compat.v1
tf.gfile = tf.io.gfile

class ObjectDetection(Node):

    def __init__(self):
        super().__init__('object_detection')
        # Declare ROS parameters
        self.declare_parameters(namespace='',
                               	parameters=[('qos_length',0),
                                            ('path_to_labels',''),
                                            ('model_name',''),
                                            ('topic.rgb_image',''),
                                            ('topic.point_cloud','')])
        qos_length = self.get_parameter('qos_length').get_parameter_value().integer_value
        qos_profile = QoSProfile(depth=qos_length,
                                 history=QoSHistoryPolicy.KEEP_LAST,
                                 reliability=QoSReliabilityPolicy.RELIABLE)
        # Load cv_bridge
        self.bridge = CvBridge()
        # Load label map
        PATH_TO_LABELS = self.get_parameter('path_to_labels').get_parameter_value().string_value
        self.category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS, use_display_name=True)
        # Load object detection model
        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.detection_model = self.loadModel(model_name)
        # Create Subscribers
        rgb_topic = self.get_parameter('topic.rgb_image').get_parameter_value().string_value
        self.rgb_sub = mf.Subscriber(self,Image,rgb_topic,qos_profile=qos_profile)
        pcd_topic = self.get_parameter('topic.point_cloud').get_parameter_value().string_value
        self.pcd_sub = mf.Subscriber(self,PointCloud2,pcd_topic,qos_profile=qos_profile)
        # Apply message filter
        self.timestamp_sync = mf.TimeSynchronizer([self.rgb_sub,self.pcd_sub],queue_size=qos_length)
        self.timestamp_sync.registerCallback(self.runInference)

    def loadModel(self,model_name):
        base_url = 'http://download.tensorflow.org/models/object_detection/'
        model_file = model_name + '.tar.gz'
        model_dir = tf.keras.utils.get_file(fname=model_name, 
                                            origin=base_url + model_file,
                                            untar=True)
        model_dir = pathlib.Path(model_dir)/"saved_model"
        model = tf.saved_model.load(str(model_dir))
        return model

    def runInference(self,rgb_msg,pcd_msg):
        image = self.bridge.imgmsg_to_cv2(rgb_msg)
        image = np.asarray(image)
        input_tensor = tf.convert_to_tensor(image)
        input_tensor = input_tensor[tf.newaxis,...]
        model_fn = self.model.signatures['serving_default']
        output_dict = model_fn(input_tensor)
        num_detections = int(output_dict.pop('num_detections'))
        output_dict = {key:value[0, :num_detections].numpy() 
                        for key,value in output_dict.items()}
        output_dict['num_detections'] = num_detections

        # detection_classes should be ints.
        output_dict['detection_classes'] = output_dict['detection_classes'].astype(np.int64)
        
        # Handle models with masks:
        if 'detection_masks' in output_dict:
            # Reframe the the bbox mask to the image size.
            detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
                    output_dict['detection_masks'], output_dict['detection_boxes'],
                    image.shape[0], image.shape[1])      
            detection_masks_reframed = tf.cast(detection_masks_reframed > 0.5,
                                            tf.uint8)
            output_dict['detection_masks_reframed'] = detection_masks_reframed.numpy()
            
        return output_dict

        def show_inference(model, image_path):
            # the array based representation of the image will be used later in order to prepare the
            # result image with boxes and labels on it.
            image_np = np.array(Image.open(image_path))
            # Actual detection.
            output_dict = run_inference_for_single_image(model, image_np)
            # Visualization of the results of a detection.
            vis_util.visualize_boxes_and_labels_on_image_array(
                image_np,
                output_dict['detection_boxes'],
                output_dict['detection_classes'],
                output_dict['detection_scores'],
                category_index,
                instance_masks=output_dict.get('detection_masks_reframed', None),
                use_normalized_coordinates=True,
                line_thickness=2)


def main(args=None):
    rclpy.init(args=args)
    det_node = LaneDetection()
    rclpy.spin(det_node)
    det_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()