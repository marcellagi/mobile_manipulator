#!/usr/bin/env python3
import rclpy
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
import tf2_ros
import numpy as np

class MatrixOperations(rclpy.node.Node):
    def __init__(self):
        super().__init__('matrix_operations')

        self.sec2_sub = self.create_subscription(Float64MultiArray, '/sec_2_topic', self.sec2_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.matrix_mult_pub = self.create_publisher(Float64MultiArray, 'matrix_mult', 10)

        self.homogeneous_matrix = None
        self.odom_matrix = None


    def sec2_callback(self, msg):
        print("sec2")
        position = np.array([msg.data[0], msg.data[1], msg.data[2]])
        orientation = np.array([0, 0, 0, 1]) 

        self.homogeneous_matrix = quaternion_to_matrix(orientation, position)
        self.perform_matrix_multiplication()


    def odom_callback(self, msg):
        print("odom")
        position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        orientation = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

        self.odom_matrix = quaternion_to_matrix(orientation, position)
        self.perform_matrix_multiplication()

    
    
    def perform_matrix_multiplication(self):
        print("mult")
        if self.homogeneous_matrix is not None and self.odom_matrix is not None:
            matrix_result = np.dot(self.homogeneous_matrix, self.odom_matrix)

            # Convertendo todos os valores para o tipo 'float'
            matrix_result = matrix_result.astype(float)

            # Limitando os valores da matriz ao intervalo aceitável para 'float'
            matrix_result = np.clip(matrix_result, -1.79e+308, 1.79e+308)

            # Convertendo a matriz para uma lista de valores 'float'
            flattened_matrix = matrix_result.flatten()
            flattened_matrix = [float(value) for value in flattened_matrix]

            msg = Float64MultiArray()
            msg.data = flattened_matrix

            self.matrix_mult_pub.publish(msg)
            self.homogeneous_matrix = None
            self.odom_matrix = None



def quaternion_to_matrix(orientation, position):
    x, y, z, w = orientation

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    rotation_matrix = np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy), 0],
        [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx), 0],
        [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy), 0],
        [0, 0, 0, 1] 
    ])

    transform_matrix = np.identity(4)
    transform_matrix[:3, :3] = rotation_matrix[:3, :3] 
    transform_matrix[:3, 3] = position

    return transform_matrix


def main(args=None):
    rclpy.init(args=args)
    print("main")
    matrix_operations = MatrixOperations()
    rclpy.spin(matrix_operations)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



