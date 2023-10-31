#!/usr/bin/env python3
import rclpy
import logging
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile

def tf_callback(msg):
    for transform in msg.transforms:
        if transform.child_frame_id == "imu_link_manipulator" and transform.header.frame_id == "manipulator_link":
            print("imu_link_manipulator em relação a manipulator_link:")
            print(f"Posição (x, y, z): {transform.transform.translation.x}, {transform.transform.translation.y}, {transform.transform.translation.z}")
        elif transform.child_frame_id == "imu_link_manipulator_2" and transform.header.frame_id == "manipulator_link":
            print("imu_link_manipulator_2 em relação a manipulator_link:")
            print(f"Posição (x, y, z): {transform.transform.translation.x}, {transform.transform.translation.y}, {transform.transform.translation.z}")

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('tf_subscriber')

    qos_profile = QoSProfile(depth=10)
    tf_subscription = node.create_subscription(
        TransformStamped,
        '/tf_static',
        tf_callback,
        qos_profile
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




