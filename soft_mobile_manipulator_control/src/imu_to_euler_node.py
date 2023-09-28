#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import numpy as np

from soft_mobile_manipulator_control.helper_funcs.curves import PCCModel, PCCModelIMU, UTISpline

def euler_from_quaternion(quaternion):
    x, y, z, w = quaternion
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class OrientationConversionNode(Node):

    def __init__(self):
        super().__init__('orientation_conversion_node')
        self.imu1_subscriber = self.create_subscription(
            Imu,
            'imu/data_raw_1',
            self.imu1_callback,
            10)
        self.imu2_subscriber = self.create_subscription(
            Imu,
            'imu/data_raw_2',
            self.imu2_callback,
            10)
        self.euler_pub1 = self.create_publisher(
            Vector3,
            'euler_orientation_imu_1_topic', 
            10)
        self.euler_pub2 = self.create_publisher(
            Vector3,
            'euler_orientation_imu_2_topic', 
            10)

    def imu1_callback(self, msg):
        euler_angles = self.orientation_to_euler(msg.orientation)
        euler_msg = Vector3()
        euler_msg.x = euler_angles[0]  
        euler_msg.y = euler_angles[1]  
        euler_msg.z = euler_angles[2]  
        self.euler_pub1.publish(euler_msg)

    def imu2_callback(self, msg):
        euler_angles = self.orientation_to_euler(msg.orientation)
        euler_msg = Vector3()
        euler_msg.x = euler_angles[0]  
        euler_msg.y = euler_angles[1]  
        euler_msg.z = euler_angles[2]  
        self.euler_pub2.publish(euler_msg)

    def orientation_to_euler(self, orientation):
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )
        euler_angles = euler_from_quaternion(quaternion)
        return euler_angles

def main(args=None):
    rclpy.init(args=args)
    node = OrientationConversionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
