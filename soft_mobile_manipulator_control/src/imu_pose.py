import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.subscription = self.create_subscription(
            Imu, '/euler_orientation_imu_1_topic', self.imu_callback_1, 10)
        self.subscription = self.create_subscription(
            Imu, '/euler_orientation_imu_2_topic', self.imu_callback_2, 10)

    def imu_callback_1(self, msg):
        imu_data = {
            'roll': msg.orientation.x,
            'pitch': msg.orientation.y,
            'yaw': msg.orientation.z
        }
        imu_state = self.get_imu_state(imu_data)

    def get_imu_state(self, imu_data):
        # Sua implementação da função get_imu_state aqui
        pass

def main(args=None):
    rclpy.init(args=args)
    my_node = MyNode()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()