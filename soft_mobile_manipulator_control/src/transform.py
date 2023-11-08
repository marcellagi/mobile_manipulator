#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, TransformStamped, Pose
from std_msgs.msg import Float64MultiArray
import numpy as np
import yaml
import os
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Twist
from time import time
from tf2_ros import TransformException
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory
from softrobots_dynamic_model.helper_funcs.plot_segment_general import PlotSegmentGeneral
from softrobots_dynamic_model.helper_funcs.curves import PCCModel, PCCModelIMU, UTISpline

class TransformListener_m(Node):

    def __init__(self):
        super().__init__('transform_listener')
        self.time = time()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_pub = self.create_publisher(TransformStamped, 'transform', 10)
        self.timer = self.create_timer(0.05, self.publishTransform)

    def publishTransform(self):
        
        try:
            transform = self.tf_buffer.lookup_transform("odom",
                                                        "imu_link_manipulator",
                                                        rclpy.time.Time())

            self.tf_pub.publish(transform)

        except TransformException as ex:
            if time() - self.time > 1.0:
                self.time = time()
                self.get_logger().info(f'Could not transform: {ex}')
            return


def main():
    rclpy.init()
    node = TransformListener_m()  # Certifique-se de passar os argumentos corretos aqui
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
