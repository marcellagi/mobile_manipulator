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

def load_yaml_file(filename) -> dict:
    """Load yaml file with the soft robot parameters."""
    with open(filename, 'r', encoding='UTF-8') as file:
        data = yaml.safe_load(file)
    return data


class OrientationConversionNode(Node):

    def __init__(self):
        super().__init__('orientation_conversion_node')
        self.imu_subscriber = self.create_subscription(
            Imu,
            'imu/data_raw',
            self.imu_callback,
            10)
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
        self.euler_pub = self.create_publisher(
            Vector3,
            'euler_orientation_imu_topic', 
            10)
        self.euler_pub1 = self.create_publisher(
            Vector3,
            'euler_orientation_imu_1_topic', 
            10)
        self.euler_pub2 = self.create_publisher(
            Vector3,
            'euler_orientation_imu_2_topic', 
            10)
        self.sec_pub1 = self.create_publisher(
            Float64MultiArray,
            'sec_1_topic', 
            10)
        self.sec_pub2 = self.create_publisher(
            Float64MultiArray,
            'sec_2_topic', 
            10)
        rate = 60

        self.time = time()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # self.tf_pub = self.create_publisher(TransformStamped, 'transform', 10)
        # self.timer = self.create_timer(0.001, self.publishTransform)


        # Load yaml file containing the soft robot parameters
        self.load_yaml_params("config_seg.yaml")

        # Initial state
        init_state = self.get_state(self.phi_list_init[:self.n_links],
                                    self.theta_list_init[:self.n_links])
        
        # Desired state
        ref_state = self.get_state(self.phi_list_target,
                                   self.theta_list_target)

        # load parameters
        self.seg_param = self.load_ms_sr(init_state, ref_state)

        # IMU
        self.subscriptions_imu = {'data_raw': np.zeros(3),
                                  'data_raw_1': np.zeros(3),
                                  'data_raw_2': np.zeros(3)}

        self._loop_root = self.create_rate(1/rate, self.get_clock())
        self.simulate()


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



    def simulate(self):
        plotter = PlotSegmentGeneral()
        pcc_model_curve, pcc_curve_data, name_pcc_from_code = self.create_pcc_curve()
        plotter.add_curve(name_pcc_from_code, pcc_curve_data)

        pcc_from_imus = self.create_pcc_model_from_imu()
        plotter.add_curve(pcc_from_imus[-1], pcc_from_imus[2])

        plotter.view()
        plotter.update_plot()

        self.current_time = 0.0
        try:
            while rclpy.ok():
                rclpy.spin_once(self)
                # Update PCC Model data
                state = np.array(
                    self.seg_param['state'][0:self.n_links*2]).reshape(-1, 1)
                pcc_curve_data = pcc_model_curve.generate_curve(state)
                plotter.update_curve_data(name_pcc_from_code, pcc_curve_data)

                state_from_imu = pcc_from_imus[1].generate_curve(
                            self.subscriptions_imu)
                pcc_curve_data_imu =\
                    pcc_from_imus[0].generate_curve(state_from_imu)
                plotter.update_curve_data(
                    pcc_from_imus[-1],
                    pcc_curve_data_imu)
                
                # Positions from IMU.
                # TODO: Compare with TF Data
                position_seg_1 = np.array(pcc_curve_data_imu)[:, 26]
                # print(f"Position seg 1 (x1, y1, z1): {position_seg_1}")
                position_seg_2 = np.array(pcc_curve_data_imu)[:, 51]
                # print(f"Position seg 2 (x2, y2, z2): {position_seg_2}")

                msg1 = Float64MultiArray()
                msg1.data = position_seg_1.tolist()

                msg2 = Float64MultiArray()
                msg2.data = position_seg_2.tolist()

                self.sec_pub1.publish(msg1)
                self.sec_pub2.publish(msg2)

                # Update plot
                plotter.update_plot()

                self.current_time += self.time_dt
        except KeyboardInterrupt:
            pass

    def get_state(
        self,
        phi_list,
        theta_list
    ) -> np.array:
        """
        Get the state of the multi segment.

        All the input are in degrees.
        It is converted to radians in the function. Theta is treated as the
        bending angle or curvature angle of the segment.

        Parameters
        ----------
        phi_list: list[list[float]]
            Each list inside the list has following parameters:
            - Orientation angle of the n-th link in degrees [phi[n]]
            - Orientation angle velocity of the n-th link in
              degrees/s [phi_dot[n]]
            - Orientation angle acceleration of the n-th link
              in degrees/s^2 [phi_ddot[n]]

        theta_list: list[list[float]])
            Each list inside the list has following parameters:
            - Inclination (bending angle) position of the n-th link
              in degrees [theta[n]]
            - Inclination (bending angle) velocity of the n-th link
              in degrees/s [theta_dot[n]]
            - Inclination (bending angle) acceleration of the n-th
              link in degrees/s^2 [theta_ddot[n]]

        Returns
        -------
        np.array
            array of shape [1, 6*num_links] containing the state of all the
            links in radians in the following order:
            - phi[0], theta[0] ... phi[n], theta[n],
            - phi[0]_dot, theta[0]_dot ... phi[n]_dot, theta[n]_dot
            - phi[0]_ddot, theta[0]_ddot ... phi[n]_ddot, theta[n]_ddot

        """
        phi_list_in_radians = np.radians(phi_list)
        theta_list_in_radians = np.radians(theta_list)
        state = np.array([])
        for state_i in range(3):
            for link_n in range(self.n_links):
                phi_i = phi_list_in_radians[link_n][state_i]
                theta_i = theta_list_in_radians[link_n][state_i]
                state = np.append(state, np.array([phi_i, theta_i]))

        return state
    
    def load_yaml_params(
        self,
        config_file_name: str
    ):
        """
        Load yaml file with the soft robot parameters.

        Parameters
        ----------
        config_file_name: string
            Yaml configuration file's name

        """
        pkg_path = get_package_share_directory('soft_mobile_manipulator_control')
        # Configuration file
        self.data = load_yaml_file(os.path.join(pkg_path,
                                                'config',
                                                config_file_name))

        self.simulation_time = self.data['simulation_time']
        self.time_dt = self.data['dt']
        self.n_links = self.data['n_links']
        self.n_joints = 5
        self.isaac_joints_per_seg = self.data['isaac_sim_joints_per_segment']

        # These are the initial state of the soft robot (phi and theta)
        self.phi_list_init = [[angle, 0, 0]
                              for angle in self.data['phi_angles_init']]
        self.theta_list_init = [[angle, 0, 0]
                                for angle in self.data['theta_angles_init']]
        # These are the target angles of the soft robot (phi and theta)
        self.phi_list_target = [[angle, 0, 0]
                                for angle in self.data['phi_angles_target']]
        self.theta_list_target =\
            [[angle, 0, 0]
             for angle in self.data['theta_angles_target']]
        
    def load_ms_sr(
        self,
        init_state: np.array,
        ref_state: np.array
    ):
        """
        Load the multi segment SR parameters.

        Parameters
        ----------
        init_state: np.array
            initial state of the multi segment
        ref_state: np.array
            reference state of the multi segment

        Returns
        -------
        dict
            multi segment SR parameters

        """
        state = {}
        state['state'] = init_state
        state['ref_state'] = ref_state

        # Parameter definitions
        state['L'] = self.data['segment_length'][:self.n_links]

        # arc length to consider in the range [0, 1]
        state['s'] = self.data['segment_normalized_size']

        # Print the SR parameters
        self.get_logger().info("SR params: " + str(state))
        return state
        
    def create_pcc_curve(self):
        """
        Create the PCC curve based on the dynamic equations.

        Returns
        -------
        pcc_model_curve: PCCModel
            PCC model based on the dynamic equations
        pcc_curve_data: np.array
            PCC curve data based on the dynamic equations
        name_pcc_from_code: str
            Name of the PCC curve based on the dynamic equations

        """
        # Instantiate PCC Model
        pcc_model_curve = PCCModel(
            self.seg_param, self.n_links)
        state = np.array(
            self.seg_param['state'][0:self.n_links*2]).reshape(-1, 1)
        pcc_curve_data = pcc_model_curve.generate_curve(state)
        # Add PCC model to the graph
        name_pcc_from_code = ""
        return pcc_model_curve, pcc_curve_data, name_pcc_from_code
    
    def create_pcc_model_from_imu(self):
        """
        Create the PCC curve based on the IMUs sensors.

        Returns
        -------
        pcc_model_curve_imu: PCCModel
            PCC model based on the dynamic equations
        pcc_model_from_imu: PCCModelIMU
            PCC model based on the IMU sensors
        pcc_curve_data_imu: np.array
            PCC curve data based on the IMU sensors
        name_pcc_from_imus: str
            Name of the PCC curve based on the IMU sensors

        """
        pcc_model_curve_imu = PCCModel(
            self.seg_param, self.n_links)

        pcc_model_from_imu = PCCModelIMU(
            self.subscriptions_imu,
            "data_raw",
            ["data_raw_1", "data_raw_2"],
            self.n_links,
            self.seg_param['L'])

        state_from_imu = pcc_model_from_imu.generate_curve(
            self.subscriptions_imu)

        # Kinematics
        pcc_curve_data_imu =\
            pcc_model_curve_imu.generate_curve(state_from_imu)

        # Add PCC model to the graph
        name_pcc_from_imus = "PCC Model - From IMUs"
        return pcc_model_curve_imu, pcc_model_from_imu, pcc_curve_data_imu, name_pcc_from_imus

    def imu_callback(self, msg):
        euler_angles = self.orientation_to_euler(msg.orientation)
        euler_msg = Vector3()
        euler_msg.x = euler_angles[0]  
        euler_msg.y = euler_angles[1]  
        euler_msg.z = euler_angles[2]  
        self.euler_pub.publish(euler_msg)
        self.subscriptions_imu['data_raw'] = np.array(euler_angles)

    def imu1_callback(self, msg):
        euler_angles = self.orientation_to_euler(msg.orientation)
        euler_msg = Vector3()
        euler_msg.x = euler_angles[0]  
        euler_msg.y = euler_angles[1]  
        euler_msg.z = euler_angles[2]  
        self.euler_pub1.publish(euler_msg)
        self.subscriptions_imu['data_raw_2'] = np.array(euler_angles)

    def imu2_callback(self, msg):
        euler_angles = self.orientation_to_euler(msg.orientation)
        euler_msg = Vector3()
        euler_msg.x = euler_angles[0]  
        euler_msg.y = euler_angles[1]  
        euler_msg.z = euler_angles[2]  
        self.euler_pub2.publish(euler_msg)
        self.subscriptions_imu['data_raw_1'] = np.array(euler_angles)

    def orientation_to_euler(self, orientation):
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )
        euler_angles = euler_from_quaternion(quaternion)
        euler_angles = np.array(euler_angles)
        euler_angles[-1] = 0
        return euler_angles

def main(args=None):
    rclpy.init(args=args)
    node = OrientationConversionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
