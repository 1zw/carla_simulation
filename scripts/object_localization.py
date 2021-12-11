import rclpy
import message_filters as mf
import numpy as np

from rclpy.node import Node
from rclpy.qos import QoSProfile,QoSHistoryPolicy,QoSReliabilityPolicy
from sensor_msgs.msg import Imu
from carla_simulation.msg import Object,ObjectArray

class ObjectLocalization(Node):

    def __init__(self):
        super().__init__('object_detection')
        # Declare ROS parameters
        self.declare_parameters(namespace='',
                               	parameters=[('qos_length',0),
                                           ])
        self.nodeParams()
        qos_length = self.get_parameter('qos_length').get_parameter_value().integer_value
        qos_profile = QoSProfile(depth=qos_length,
                                 history=QoSHistoryPolicy.KEEP_LAST,
                                 reliability=QoSReliabilityPolicy.RELIABLE)
        # Create Subscribers
        imu_topic = self.get_parameter('topic.imu').get_parameter_value().string_value
        self.imu_sub = mf.Subscriber(self,Imu,imu_topic,qos_profile=qos_profile)
        obj_topic = self.get_parameter('topic.tracked_obj').get_parameter_value().string_value
        self.obj_sub = mf.Subscriber(self,ObjectArray,obj_topic,qos_profile=qos_profile)
        # Apply message filter
        self.timestamp_sync = mf.TimeSynchronizer([self.imu_sub,self.obj_sub],queue_size=qos_length)
        self.timestamp_sync.registerCallback(self.imuCallback)

    def imgCallback(self,imu_msg,obj_msg):
        pass
        
