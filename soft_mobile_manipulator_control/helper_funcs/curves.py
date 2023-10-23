# Copyright (c) 2023, SENAI Cimatec
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from abc import abstractmethod

import numpy as np
from ament_index_python.packages import get_package_share_directory
from scipy import integrate
from scipy.interpolate import CubicSpline

from softrobots_dynamic_model.helper_funcs.load_sym_functions import \
    LoadSymbolic
from softrobots_dynamic_model.helper_funcs.kinematics import \
    rotation_matrix_x, rotation_matrix_y, rotation_matrix_z


class GeneratelCurve():
    """General abstract class to implement new curves."""

    @abstractmethod
    def generate_curve(self, params):
        """
        Generate curve to plot the robot shape.

        It must return a np.array of shape (3, n) where n is the number of
        points of the curve.
        The number of parameters of this method depends on the data needed to
        generate the curve.

        Parameters
        ----------
        params : dict or list or np.array
            List of parameters needed to generate the curve.

        """


class PCCModel(GeneratelCurve):
    """Plot each segment of the soft robot."""

    def __init__(
        self,
        seg_param: dict,
        n_links: int
    ):
        """
        Initialize parameters to plot the soft robot using matplotlib.

        Parameters
        ----------
        seg_param : dict
            Segment parameters.
        n_links : int
            Number of links.

        """
        self.pkg_path = get_package_share_directory('softrobots_dynamic_model')

        # Load symbolic functions - teste
        self.sym_functions = LoadSymbolic(n_links)

        self.n_links = n_links
        self.state = np.array(seg_param['state'][0:n_links*2]).reshape(-1, 1)
        self.ref_state = np.array(
            seg_param['ref_state'][0:n_links*2]).reshape(-1, 1)

        self.n_seg = int(self.state.shape[0]/2)
        self.n_points = 25  # number of points to plot one segment
        self.z_direction = 1
        self.seg = np.zeros((3, self.n_points, self.n_seg))
        self.segbar = np.zeros((3, self.n_points, self.n_seg))
        self.length = seg_param['L']

    def generate_curve(
        self,
        params: np.array
    ):
        """
        Join n segments before plotting.

        It overrides the method from the GeneratelCurve abstract class.

        Parameters
        ----------
        params: np.array
            Column vector - [phi0, theta0, phi1, theta1, ..., phin, thetan].T

        """
        for j in range(self.n_points):
            seg_i = (j+1) / self.n_points
            for seg in range(self.n_seg):
                if seg == 0:
                    sid = 1
                    fk0 = self.sym_functions.kinematic_matrices.fk_dill(
                        *params[0:2].T[0],
                        self.length[0], 1).squeeze()
                    self.seg[:, j, 0] =\
                        self.sym_functions.kinematic_matrices.fk_dill(
                            *params[0:2].T[0],
                            self.length[0], seg_i).squeeze()
                else:
                    sid += 1
                    fk_total = np.zeros((3, 1))
                    for i in range(0, seg):
                        rotation_matrix = np.eye(3)

                        for k in range(0, i+1):
                            rotation_matrix =\
                                rotation_matrix @\
                                self.sym_functions.kinematic_matrices.rot_dill(
                                    *params[k*2:(k+1)*2].T[0], 1)

                        s_len = seg_i if sid-1 == i+1 else 1
                        fk_total += rotation_matrix @\
                            self.sym_functions.kinematic_matrices.fk_dill(
                                *params[(i+1)*2:(i+2)*2].T[0],
                                self.length[i+1], s_len)

                    self.seg[:, j, seg] = fk0 + fk_total.T

        seg_plot_list = []
        seg_plot_list.append(np.hstack((np.zeros((3, 1)), self.seg[:, :, 0])))
        for link_n in range(1, self.n_seg):
            seg_plot_list.append(np.hstack(
                (self.seg[:, -1, link_n-1].reshape(-1, 1),
                 self.seg[:, :, link_n])))

        seg_plot_array = np.hstack(seg_plot_list)
        return seg_plot_array


class UTISpline(GeneratelCurve):
    """Plot each segment of the soft robot."""

    def __init__(
        self,
        subscriptions_imu: dict,
        imu_base_name: str,
        imu_seg_names: list,
        n_links: int,
        segment_lengths: list,
    ):
        self.subscriptions_imu = subscriptions_imu
        self.imu_base_name = imu_base_name
        self.imu_seg_names = imu_seg_names
        self.n_links = n_links
        self.length = segment_lengths
        # Since all segments have the same length, we can
        # use the lengt[0]
        self.n_of_imus = len(self.imu_seg_names) + 1
        self.size_of_segment = self.n_links*self.length[0]

        self.abs_link = self.size_of_segment
        self.x_lim = [-self.size_of_segment, self.size_of_segment]
        self.y_lim = [-self.size_of_segment, self.size_of_segment]
        self.z_lim = [-self.size_of_segment, self.size_of_segment]

    def get_imu_state(
        self,
        imu_subscription: dict
    ) -> list:
        """
        Get the IMU state.

        Parameters
        ----------
        imu_subscription: dict
            Dictionary with the IMU data.

        Returns
        -------
        list
            List with the IMU data with the following order:
            [imu_rot_x, imu_rot_y, imu_rot_z]

        """
        states_imu = []
        # Stores the IMU data in the following order:
        # [imu_rot_x, imu_rot_y, imu_rot_z]
        # The Iimu_rot_z MU_SENSOR_SEG_0 is the
        # z rotation of the base IMU
        for link_n in range(self.n_links):
            if link_n == 0:
                # Append angle around X and Y of the next segment
                # And the angle around Z of the base IMU
                states_imu.append([
                    imu_subscription[self.imu_seg_names[link_n]][0],
                    imu_subscription[self.imu_seg_names[link_n]][1],
                    imu_subscription[self.imu_seg_names[link_n]][2]
                    # imu_subscription[self.imu_base_name][-1]
                ])
            else:
                # Append the angle around X, Y and Z of the next segment
                states_imu.append(
                    imu_subscription[self.imu_seg_names[link_n]])
        return states_imu

    def euler_to_univec(self, euler_angles: float):
        """
        Transform euler angles to unit vector.

        Parameters
        ----------
        euler_angles : float
            Euler angles in radians.

        Returns
        -------
        np.array
            Unit vector.

        """
        roll_rad, pitch_rad, yaw_rad = euler_angles

        # Create rotation matrices
        rotation_yaw = np.array([[np.cos(yaw_rad), -np.sin(yaw_rad), 0],
                                [np.sin(yaw_rad), np.cos(yaw_rad), 0],
                                [0, 0, 1]])

        rotation_pitch = np.array([[np.cos(pitch_rad), 0, np.sin(pitch_rad)],
                                   [0, 1, 0],
                                   [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]])

        rotation_roll = np.array([[1, 0, 0],
                                  [0, np.cos(roll_rad), -np.sin(roll_rad)],
                                  [0, np.sin(roll_rad), np.cos(roll_rad)]])

        # Initial reference vector
        direction_vector = np.array([0, 0, -1])

        # Apply rotations
        direction_vector = np.dot(rotation_yaw, direction_vector)
        direction_vector = np.dot(rotation_pitch, direction_vector)
        direction_vector = np.dot(rotation_roll, direction_vector)

        # Normalize the resulting vector to unit length
        direction_vector /= np.linalg.norm(direction_vector)

        return direction_vector

    def generate_curve(
        self,
        params: dict
    ) -> np.array:
        """
        Generate curve to plot the robot shape.

        It overrides the method from the GeneratelCurve abstract class.

        The number of parameters of this method depends on the data needed to
        generate the curve.

        Parameters
        ----------
        params : dict
            Dictionary with the IMU data.

        Returns
        -------
        np.array
            array of shape (3, n) where n is the number of points of the curve.

        """
        states_imu = []
        for key, value in params.items():
            states_imu.append(value)

        univec_list = []
        for imu_index in range(self.n_of_imus):
            rad_angles = states_imu[imu_index]
            univec = self.euler_to_univec(rad_angles)
            univec_list.append(univec)

        points = np.zeros((7, 3))
        for imu_index in range(self.n_of_imus):
            points[imu_index, :] = np.array(univec_list[imu_index])

        ts_var = np.linspace(0, self.size_of_segment, num=7, endpoint=True)
        tn_cs = []
        for imu_index in range(self.n_of_imus):
            cubic = CubicSpline(ts_var, points[:, imu_index])
            tn_cs.append(cubic)

        # New x values for interpolation
        ts_interp = np.linspace(0, self.size_of_segment, 10)
        tn_interp = []
        for imu_index in range(self.n_of_imus):
            tn_interp.append(tn_cs[imu_index](ts_interp))

        segment_points = []
        for imu_index in range(self.n_of_imus):
            seg_p = integrate.cumtrapz(tn_interp[imu_index],
                                       x=ts_interp,
                                       axis=0,
                                       initial=0)
            segment_points.append(seg_p)

        segment_points_array = np.hstack(segment_points).reshape(3, -1)
        return segment_points_array


class PCCModelIMU(GeneratelCurve):
    """Plot each segment of the soft robot."""

    def __init__(
        self,
        subscriptions_imu: dict,
        imu_base_name: str,
        imu_seg_names: list,
        n_links: int,
        segment_lengths: list,
    ):
        """
        Initialize parameters to plot the soft robot using matplotlib.

        Parameters
        ----------
        subscriptions_imu : dict
            Dictionary with the IMU data.
        imu_base_name : str
            Name of the base IMU.
        imu_seg_names : list
            List with the IMU names.
        n_links : int
            Number of links.
        segment_lengths : list
            List with the length of each segment.

        """
        self.subscriptions_imu = subscriptions_imu
        self.imu_base_name = imu_base_name
        self.imu_seg_names = imu_seg_names
        self.n_links = n_links
        self.length = segment_lengths
        self.num_points_to_plot = 10

    def get_imu_state(
        self,
        imu_subscription: dict
    ) -> list:
        """
        Get the IMU state.

        Parameters
        ----------
        imu_subscription: dict
            Dictionary with the IMU data.

        Returns
        -------
        list
            List with the IMU data with the following order:
            [imu_rot_x, imu_rot_y, imu_rot_z]

        """
        states_imu = []
        # Stores the IMU data in the following order:
        # [imu_rot_x, imu_rot_y, imu_rot_z]
        for link_n in range(self.n_links):
            if link_n == 0:
                # Append angle around X and Y of the next segment
                # And the angle around Z of the base IMU
                states_imu.append([
                    imu_subscription[self.imu_seg_names[link_n]][0],
                    imu_subscription[self.imu_seg_names[link_n]][1],
                    imu_subscription[self.imu_seg_names[link_n]][2]
                ])
            else:
                # Append the angle around X, Y and Z of the next segment
                states_imu.append(
                    list(imu_subscription[self.imu_seg_names[link_n]]))
        return states_imu

    def generate_curve(
        self,
        params: dict
    ):
        """
        Get the state data using IMUs.

        It overrides the method from the GeneratelCurve abstract class.

        Parameters
        ----------
        params: dict
            Dictionary with the IMU data.

        Returns
        -------
        np.array
            Column vector - [phi0, theta0, phi1, theta1, ..., phin, thetan].T

        """
        states_imu = self.get_imu_state(params)
        rot_mat_world_zyx = [np.eye(3)]*self.n_links
        rot_mat_reconstructed = [np.eye(3)]*self.n_links
        state = np.zeros((self.n_links*2, 1))

        phi_list = [0]*self.n_links
        number_of_imus = len(states_imu)
        for imu_index in range(number_of_imus):
            state_imu = states_imu[imu_index]
            rotation_x = rotation_matrix_x(state_imu[0])
            rotation_y = rotation_matrix_y(state_imu[1])
            rotation_z = rotation_matrix_z(state_imu[2])

            # Rotation matrices global are calculated
            # with respect to the world frame
            # rot_mat_world_zyx = rotation_x @ rotation_y @ rotation_z
            rot_mat_world_zyx = rotation_z @ rotation_y @ rotation_x

            # Calculate the phi angle or the orientation (rotation around Z)
            if imu_index == 0:
                z_proj_prev_frame = rot_mat_world_zyx[:, 2]
                print(f"z_proj_prev_frame[0] [{imu_index}]: {z_proj_prev_frame[0]}")
                print(f"z_proj_prev_frame[1] [{imu_index}]: {z_proj_prev_frame[1]}")
                phi = np.arctan2(z_proj_prev_frame[1], z_proj_prev_frame[0])

                # Theta signal is necessary to keep the theta angle between
                # -pi/2 and pi/2, according to the phi angle
                # NOTE: the signal must be after the phi calculation without
                # correction
                theta_signal = -1 if phi > np.pi/2 or phi < -np.pi/2 else 1

                phi = self.fix_phi_angle(phi)
                print(f"Phi [{imu_index}]: {phi}")
                phi_list[imu_index] = phi

                # It is necessary to normalize the third column of the
                # rotation matrix to avoid floating point approximation
                # errors
                length = np.linalg.norm(rot_mat_world_zyx[:, 2])
                normalized_third_column = rot_mat_world_zyx[:, 2] / length
                theta = np.arccos(normalized_third_column[2]) * theta_signal
                print(f"Theta [{imu_index}]: {theta}")

                # We need to reconstruct the homogeneous transformation matrix
                # based on the phi and theta angles because the orientation
                # of the next segment affects the orientation of the previous
                # segment and we loose reference
                rotation_y = rotation_matrix_y(theta)
                rotation_z = rotation_matrix_z(phi)
                rot_mat_reconstructed[imu_index] = rotation_z @ rotation_y
            else:
                # Local rotation matrices are calculated
                # with respect to the previous segment
                rot_mat_local =\
                    np.linalg.inv(rot_mat_reconstructed[imu_index-1]) \
                    @ rot_mat_world_zyx
                z_proj_prev_frame = rot_mat_local[:, 2]
                phi = np.arctan2(z_proj_prev_frame[1], z_proj_prev_frame[0])

                # Theta signal is necessary to keep the theta angle between
                # -pi/2 and pi/2, according to the phi angle
                # NOTE: the signal must be after the phi calculation without
                # correction
                theta_signal = -1 if phi > np.pi/2 or phi < -np.pi/2 else 1

                # Sum all previous phi angles
                if imu_index < number_of_imus - 1:
                    for phi_i in range(0, imu_index):
                        phi += phi_list[phi_i]

                phi = self.fix_phi_angle(phi)
                phi_list[imu_index] = phi

                # It is necessary to normalize the third column of the
                # rotation matrix to avoid floating point approximation
                # errors
                length = np.linalg.norm(rot_mat_local[:, 2])
                normalized_third_column = rot_mat_local[:, 2] / length
                theta = np.arccos(normalized_third_column[2]) * theta_signal
                rot_mat_reconstructed[imu_index] = rot_mat_world_zyx

            state[imu_index*2:imu_index*2+2] =\
                np.array([phi, theta]).reshape(-1, 1)

        return state

    def fix_phi_angle(self, phi):
        """
        Fix the phi angle.

        The phi angle must be between -pi/2 and pi/2.

        Parameters
        ----------
        phi : float
            Angle in radians.

        Returns
        -------
        float
            Angle in radians.

        """
        if phi > np.pi/2:
            phi = phi - np.pi
        elif phi < -np.pi/2:
            phi = phi + np.pi
        return phi
