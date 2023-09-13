import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg

class TF2Publisher(Node):

    def __init__(self):
        super().__init__('tf2_publisher')
        self.br = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(1.0, self.publish_transforms)

    def publish_transforms(self):
        # Crie e publique a primeira transformação
        t1 = geometry_msgs.msg.TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = 'manipulator_link'
        t1.child_frame_id = 'cobra_body_3_joint'
        t1.transform.translation.x = 1.0
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = 0.0
        t1.transform.rotation.w = 1.0
        self.br.sendTransform(t1)

        # Crie e publique a segunda transformação
        t2 = geometry_msgs.msg.TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'base_link'
        t2.child_frame_id = 'link2'
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 1.0
        t2.transform.translation.z = 0.0
        t2.transform.rotation.w = 1.0
        self.br.sendTransform(t2)

        # Crie e publique a terceira transformação
        t3 = geometry_msgs.msg.TransformStamped()
        t3.header.stamp = self.get_clock().now().to_msg()
        t3.header.frame_id = 'base_link'
        t3.child_frame_id = 'link3'
        t3.transform.translation.x = 0.0
        t3.transform.translation.y = 0.0
        t3.transform.translation.z = 1.0
        t3.transform.rotation.w = 1.0
        self.br.sendTransform(t3)

def main(args=None):
    rclpy.init(args=args)
    tf2_publisher = TF2Publisher()
    rclpy.spin(tf2_publisher)
    tf2_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
