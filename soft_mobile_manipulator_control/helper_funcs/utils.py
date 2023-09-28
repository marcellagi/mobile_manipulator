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

import copy

import numpy as np
from geometry_msgs.msg import Quaternion
from numpy.linalg import inv, pinv
from scipy.integrate import solve_ivp

# The code below is from Robotarium team. It is not relevant for the code.
# TODO: Check with the Robotarium team the necessity of keeping the code
# below
# def tau_map(state_q):
#   phi = state_q[0]
#   theta = state_q[1]
#   matrix_a = np.array([[-np.cos(phi)*np.sin(theta), -np.sin(phi)],
#                        [-np.sin(phi)*np.sin(theta), np.cos(phi)]])
#   return matrix_a


def kinematic_control(state: dict, jac_rot: np.array, fk_rot: np.array,
                      grav_matrix: np.array, data: dict):
    """
    Inverse velocity kinematic control of the multi segment SR based.

    Parameters
    ----------
    state: dict
        Multi segment SR parameters.
    jac_rot: np.array
        Jacobian of the rotation matrix of the multi segment.
    fk_rot: np.array
        Forward kinematics of the multi segment robot.
    grav_matrix: np.array
        Gravity matrix of the multi segment.
    data: dict
        Data dictionary containing all the simulation parameters.

    """
    time_dt = data['dt']
    des_pos = np.array(data["target_position"])
    n_links = data["n_links"]
    cur_state = state['state'][0:2*n_links]
    ref_state = state['ref_state'][0:2*n_links]
    phi_vec = ref_state[::2]
    theta_vec = ref_state[1::2]

    def qdot_input(time, q_bar, phi_vec, theta_vec, state, des_pos, j_rot,
                   fk_rot):
        phi_vec = q_bar[::2]
        theta_vec = q_bar[1::2]
        qdot_input =\
            pinv(j_rot(*phi_vec, *theta_vec, *state['L'], state['s'][0])) @\
            (state['Ke'] * (des_pos - fk_rot(*phi_vec, *theta_vec, *state['L'],
                                             state['s'][0])))
        return qdot_input

    sol = solve_ivp(qdot_input,
                    t_span=[0, time_dt],
                    args=(phi_vec, theta_vec, state, des_pos.reshape(
                        3, 1), jac_rot, fk_rot),
                    y0=ref_state,
                    rtol=data['solver']['rtol'], atol=data['solver']['atol'],
                    method=data['solver']['method'])

    qdot_input_sol = sol.y
    qbar_end = qdot_input_sol[:, -1]  # OK

    # The code below is from Robotarium team. They will probably integrate the
    # velocity in the future to acquire the segment's position. They left this
    # as a comment to help future implementations.
    # We kept it because all the inputs were compared to the matlab's code and
    # if they decide to use this in the future, we won't need to redo the work.
    # qdot_u = qdot_input(time_dt, qbar_end, phi_vec, theta_vec, state,
    # des_pos.reshape(3, 1), jac_rot, fk_rot)

    phi_vec = qbar_end[::2]
    theta_vec = qbar_end[1::2]
    tau = (state['K'] @ qbar_end).reshape((n_links*2, 1))\
        + grav_matrix(*phi_vec, *theta_vec, *state['L'], *state['m'],
                      *state['g'].T)\
        + np.multiply(state['Kp'], (qbar_end -
                      cur_state).reshape(1, n_links*2)).T
    state['ref_state'] = qbar_end

    return tau, state


def calc_tau(state: dict, n_links: int, grav_matrix: np.array):
    """
    Calculate the torque applied to the multi segment SR.

    Parameters
    ----------
    state: dict
        Multi segment SR parameters.
    n_links: int
        Number of links in the multi segment.
    grav_matrix: np.array
        Gravity matrix of the multi segment.

    Returns
    -------
    np.array
        Torque applied to the multi segment. Shape (2*n_links,1)

    """
    state_q = state['state'][0:2*n_links]
    state_ref = state['ref_state'][0:2*n_links]
    state_q_dot = state['state'][2*n_links:4*n_links]
    state_ref_dot = state['ref_state'][2*n_links:4*n_links]
    phi_vec = state_ref[::2]
    theta_vec = state_ref[1::2]

    # The code below is from Robotarium team. They kept the code below as a
    # comment to help future implementations.
    # TODO: Check with the Robotarium team the necessity of keeping the code
    # below

    # a_matrices = []
    # for link_n in range(n_links):
    # A_qn matrices, [0,2], [2,4], [4,6]
    # a_qn = tau_map(state['state'][link_n*2:(link_n+1)*2])
    # a_matrices.append(a_qn)

    # a_q_diag = np.array([])
    # for link_n in range(n_links):
    #     a_q_diag_row = np.zeros((2, n_links*2))
    #     a_q_diag_row[:, link_n*2:(link_n+1)*2] = a_matrices[link_n]
    # append A_q_diag_row vertically to A_q_diag
    # a_q_diag = np.vstack((a_q_diag, a_q_diag_row)
    #   ) if a_q_diag.size else a_q_diag_row

    if n_links == 1:
        # Kd gain is not considered when 1 link is used
        tau_ref = (state['K'] @ state_ref.T).reshape((n_links*2, 1))\
            + grav_matrix(*phi_vec, *theta_vec, *state['L'], *state['m'],
                          *state['g'].T)\
            + (state['Kp']*(state_ref-state_q)).reshape((2*n_links, 1))
    else:
        tau_ref = (state['K'] @ state_ref.T).reshape((n_links*2, 1))\
            + grav_matrix(*phi_vec, *theta_vec, *state['L'], *state['m'],
                          *state['g'].T)\
            + (state['Kp']*(state_ref-state_q)).reshape((2*n_links, 1))\
            + (state['Kd'] * (state_ref_dot-state_q_dot)
               ).reshape((2*n_links, 1))

    # Noticed that tau is equal to tau_ref, since a_q_diag * inv(a_q_diag) = I
    # However, we keep the code below as a comment to help future
    # implementations, since it is still being investidated.

    # tau_a = inv(a_q_diag) @ tau_ref
    # tau = a_q_diag @ tau_a
    return tau_ref


def ms_sr_dynamics(state: dict, tau: np.array,
                   mass_matrix: np.array, coriolis_matrix: np.array,
                   gravity_matrix: np.array, data: dict):
    """
    Dynamics of the multiple segment SR.

    Parameters
    ----------
    state: dict
        Multi segment SR parameters.
    tau: np.array
        Torque applied to the multi segment.
    mass_matrix: np.array
        Mass matrix of the multi segment.
    coriolis_matrix: np.array
        Coriolis matrix of the multi segment.
    gravity_matrix: np.array
        Gravity matrix of the multi segment.
    data: dict
        Data dictionary containing all the simulation parameters.

    Returns
    -------
    sr: dict
        Multi segment SR parameters.

    """
    time_dt = data['dt']
    n_links = data['n_links']

    # State position
    state_q = copy.deepcopy(state['state'][0:2*n_links])
    phi_vec = state_q[::2]
    theta_vec = state_q[1::2]
    # State velocity
    q_dot = state['state'][2*n_links:4*n_links]
    phi_dot_vec = q_dot[::2]
    theta_dot_vec = q_dot[1::2]

    for idx, (theta, theta_dot) in enumerate(zip(theta_vec, theta_dot_vec)):
        if theta < 0.001 and theta > -0.001:
            theta = 0.001*np.sign(theta_dot)
        elif theta > 200:
            message = (f"\n\nq[{idx}] is too large: {theta}\n"
                       "Please consider increasing the integration step"
                       " or verifying that the desired theta value is"
                       " not zero."
                       "\nThis issue might occur if the applied torque"
                       " is zero.")
            raise ValueError(message)

    state['G'] = gravity_matrix(
        *phi_vec, *theta_vec, *state['L'], *state['m'], *state['g'].T)
    state['M'] = mass_matrix(*phi_vec, *theta_vec, *state['L'], *state['m'])
    state['C'] = coriolis_matrix(*phi_vec, *theta_vec, *phi_dot_vec,
                                 *theta_dot_vec, *state['L'], *state['m'])

    # Define the dynamics function
    def q_ddot_(time, q_dot, inverted_mass_matrix, coriolis_matrix, grav_matrix,
                stiffness_mat, state_q, damping_mat, tau):
        # The M matrix must be inverted before passing it to this function to
        # increase efficiency
        q_ddot_ = inverted_mass_matrix @\
            (-((coriolis_matrix @ q_dot).reshape(2*n_links, 1) + grav_matrix
               + (stiffness_mat @ state_q).reshape((2*n_links, 1))
               + (damping_mat @ q_dot).reshape((2*n_links, 1)) - tau))
        q_ddot_ = q_ddot_.reshape((2*n_links,))
        return q_ddot_

    sol = solve_ivp(q_ddot_,
                    t_span=[0, time_dt],
                    y0=q_dot,
                    args=(inv(state['M']), state['C'], state['G'],
                          state['K'], state_q, state['D'], tau),
                    rtol=data['solver']['rtol'], atol=data['solver']['atol'],
                    method=data['solver']['method'])
    q_dot = sol.y
    q_dot_end = q_dot[:, -1]  # OK

    # Acceleration
    state['state'][4*n_links:6*n_links] = q_ddot_(time_dt, q_dot_end,
                                                  inv(state['M']),
                                                  state['C'], state['G'],
                                                  state['K'], state_q,
                                                  state['D'], tau)
    # Velocity
    state['state'][2*n_links:4*n_links] = q_dot_end
    # Position
    state['state'][0:2*n_links] = state_q + q_dot_end*time_dt
    return state


def quaternion_to_euler(
    quaternion: Quaternion,
    radians: bool = True
) -> np.array:
    """
    Convert quaternion to degrees.

    Parameters
    ----------
    quaternion: Quaternion
        ROS Quaternion Msg to be converted
    radians: bool
        If True, the output will be in radians. If False, the output will be
        in degrees.

    Returns
    -------
    np.array
        Quaternion converted to degrees.

    """
    quaternion = np.array([quaternion.x,
                           quaternion.y,
                           quaternion.z,
                           quaternion.w])

    x = quaternion[0]
    y = quaternion[1]
    z = quaternion[2]
    w = quaternion[3]

    ori_angles = []

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    ori_angles.append(np.arctan2(t0, t1))  # roll_x

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    ori_angles.append(np.arcsin(t2))  # pitch_y

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    ori_angles.append(np.arctan2(t3, t4))  # yaw_z

    ori_angles = np.array(ori_angles)
    if not radians:
        ori_angles = np.degrees(ori_angles)

    return ori_angles
