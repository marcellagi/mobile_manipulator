#!/usr/bin/env python3
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

import argparse
import os
from typing import Tuple

import dill
from sympy import (Matrix, MutableDenseNDimArray, cos, diff, eye, integrate,
                   lambdify, simplify, sin, symbols, zeros)


def get_modified_translation_and_rotation_matrices_z_up(
    theta_vec: tuple,
    phi_vec: tuple,
    n_seg: int,
    segment_len: list,
    norm_len_s: symbols
) -> Tuple[list, list]:
    """
    Generate a modified version of the translation and rotation matrices.

    pointing in the +z direction. When theta is positive, the robot bends 
    to the +x direction.

    Parameters
    ----------
    theta_vec : Tuple[Symbol]
        Tuple of symbolic theta angles.
    phi_vec : Tuple[Symbol]
        Tuple of symbolic phi angles.
    n_seg : int
        Number of segments.
    segment_len : list[Matrix]
        List of symbolic segment lengths.
    norm_len_s : Symbol
        Normalized length of the segment.

    Returns
    -------
    transl_vec : list
        List of symbolic translation vectors.
    rotation_matrix : list
        List of symbolic rotation matrices.

    """
    # Translation vector
    transl_vec = Matrix([
        segment_len[n]/theta_vec[n] * Matrix([
            (1 - cos(norm_len_s*theta_vec[n]))*cos(phi_vec[n]),
            (1 - cos(norm_len_s*theta_vec[n]))*sin(phi_vec[n]),
            sin(norm_len_s*theta_vec[n])
        ]) for n in range(n_seg)
    ]).reshape(n_seg, 3)

    # Rotation matrix
    rotation_matrix = Matrix([
        Matrix([
          [
            sin(phi_vec[n])**2 + cos(phi_vec[n])**2*cos(norm_len_s*theta_vec[n]),
            (cos(norm_len_s*theta_vec[n]) - 1)*sin(phi_vec[n])*cos(phi_vec[n]),
            sin(norm_len_s*theta_vec[n])*cos(phi_vec[n])
          ],
          [
            (cos(norm_len_s*theta_vec[n]) - 1)*sin(phi_vec[n])*cos(phi_vec[n]),
            sin(phi_vec[n])**2*cos(norm_len_s*theta_vec[n]) + cos(phi_vec[n])**2,
            sin(phi_vec[n])*sin(norm_len_s*theta_vec[n])
          ],
          [
            -sin(norm_len_s*theta_vec[n])*cos(phi_vec[n]),
            -sin(phi_vec[n])*sin(norm_len_s*theta_vec[n]),
            cos(norm_len_s*theta_vec[n])
          ]
        ])
        for n in range(n_seg)])
    return transl_vec, rotation_matrix


def get_default_translation_and_rotation_matrices(
    theta_vec: tuple,
    phi_vec: tuple,
    n_seg: int,
    segment_len: list,
    norm_len_s: symbols
) -> Tuple[list, list]:
    """
    Generate a modified version of the translation and rotation matrices.

    The segment is already point in the -z direction and if theta is
    positive, it bends towards the -x direction.
    These matrices must be used with gravity 9.81 m/s^2 and positive
    z axis when plotting. OBS: this is preferred over changing gravity
    direction and inverting z when plotting because the mass matrix is
    calculated using the modified translation and rotation matrices.

    Parameters
    ----------
    theta_vec : Tuple[Symbol]
        Tuple of symbolic theta angles.
    phi_vec : Tuple[Symbol]
        Tuple of symbolic phi angles.
    n_seg : int
        Number of segments.
    segment_len : list[Matrix]
        List of symbolic segment lengths.
    norm_len_s : Symbol
        Normalized length of the segment.

    Returns
    -------
    transl_vec : list
        List of symbolic translation vectors.
    rotation_matrix : list
        List of symbolic rotation matrices.

    """
    # Translation vector
    transl_vec = Matrix([
        segment_len[n]/theta_vec[n] * Matrix([
            (cos(phi_vec[n]) * (cos(norm_len_s*theta_vec[n]) - 1)),
            sin(phi_vec[n]) * (cos(norm_len_s*theta_vec[n]) - 1),
            sin(norm_len_s*theta_vec[n])
        ]) for n in range(n_seg)
    ]).reshape(n_seg, 3)

    # Rotation matrix
    rotation_matrix = Matrix([
        Matrix([
            [
                sin(phi_vec[n])**2 + cos(phi_vec[n])**2 *
                cos(norm_len_s*theta_vec[n]),
                (cos(norm_len_s*theta_vec[n]) - 1) *
                sin(phi_vec[n])*cos(phi_vec[n]),
                -sin(norm_len_s*theta_vec[n])*cos(phi_vec[n])
            ],
            [
                (cos(norm_len_s*theta_vec[n]) - 1) *
                sin(phi_vec[n])*cos(phi_vec[n]),
                sin(phi_vec[n])**2*cos(norm_len_s *
                                       theta_vec[n]) + cos(phi_vec[n])**2,
                -sin(phi_vec[n])*sin(norm_len_s*theta_vec[n])
            ],
            [
                sin(norm_len_s*theta_vec[n])*cos(phi_vec[n]),
                sin(phi_vec[n])*sin(norm_len_s*theta_vec[n]),
                cos(norm_len_s*theta_vec[n])
            ]
        ])
        for n in range(n_seg)])
    return transl_vec, rotation_matrix


def get_modified_translation_and_rotation_matrix(
    theta_vec: tuple,
    phi_vec: tuple,
    n_seg: int,
    segment_len: list,
    norm_len_s: symbols
) -> Tuple[list, list]:
    """
    Generate a modified version of the translation and rotation matrices.

    The segment is already point in the -z direction and if theta is
    positive, it bends towards the -x direction.
    These matrices must be used with gravity 9.81 m/s^2 and positive
    z axis when plotting. OBS: this is preferred over changing gravity
    direction and inverting z when plotting because the mass matrix is
    calculated using the modified translation and rotation matrices.

    Parameters
    ----------
    theta_vec : Tuple[Symbol]
        Tuple of symbolic theta angles.
    phi_vec : Tuple[Symbol]
        Tuple of symbolic phi angles.
    n_seg : int
        Number of segments.
    segment_len : list[Matrix]
        List of symbolic segment lengths.
    norm_len_s : Symbol
        Normalized length of the segment.

    Returns
    -------
    transl_vec : list
        List of symbolic translation vectors.
    rotation_matrix : list
        List of symbolic rotation matrices.

    """
    # Translation vector
    transl_vec = Matrix([
        segment_len[n]/theta_vec[n] * Matrix([
            cos(phi_vec[n])*(cos(norm_len_s*theta_vec[n]) - 1),
            sin(phi_vec[n])*(cos(norm_len_s*theta_vec[n]) - 1),
            -sin(norm_len_s*theta_vec[n])
        ]) for n in range(n_seg)
    ]).reshape(n_seg, 3)

    # Rotation matrix
    rotation_matrix = Matrix([
        Matrix([
            [
                sin(phi_vec[n])**2 + cos(phi_vec[n])**2 *
                cos(norm_len_s*theta_vec[n]),
                (cos(norm_len_s *
                 theta_vec[n]) - 1)*sin(phi_vec[n])*cos(phi_vec[n]),
                sin(norm_len_s*theta_vec[n])*cos(phi_vec[n])
            ],
            [
                (cos(norm_len_s *
                 theta_vec[n]) - 1)*sin(phi_vec[n])*cos(phi_vec[n]),
                sin(phi_vec[n])**2*cos(norm_len_s *
                                       theta_vec[n]) + cos(phi_vec[n])**2,
                sin(phi_vec[n])*sin(norm_len_s*theta_vec[n])
            ],
            [
                -sin(norm_len_s*theta_vec[n])*cos(phi_vec[n]),
                -sin(phi_vec[n])*sin(norm_len_s*theta_vec[n]),
                cos(norm_len_s*theta_vec[n])
            ]
        ])
        for n in range(n_seg)])

    return transl_vec, rotation_matrix


def generate_n_segment_symbolic_functions(n_seg: int):
    """
    Generate the symbolic functions for the n-segment system.

    This script will save the G, C and M matrices as files in the
    folder "generated_sym_functions/{num_segments}_segment_python".

    Parameters
    ----------
    n_seg: int
        Number of segments.

    """
    theta_vec = symbols(" ".join([f'theta{n}' for n in range(n_seg)]),
                        nonzero=True)
    phi_vec = (symbols(" ".join([f'phi{n}' for n in range(n_seg)])))
    theta_dot_vec = (
        symbols(" ".join([f'theta_dot{n}' for n in range(n_seg)])))
    phi_dot_vec = (symbols(" ".join([f'phi_dot{n}' for n in range(n_seg)])))
    length_vec = (symbols(" ".join([f'L{n}' for n in range(n_seg)])))
    m_vec = (symbols(" ".join([f'm{n}' for n in range(n_seg)])))

    if n_seg == 1:
        theta_dot_vec = (theta_dot_vec,)
        theta_vec, phi_vec = (theta_vec,), (phi_vec,)
        phi_dot_vec, length_vec, m_vec = (
            phi_dot_vec,), (length_vec,), (m_vec,)

    norm_len_s, grav_x, grav_y, grav_z = symbols('s gx gy gz')
    grav = Matrix([grav_x, grav_y, grav_z]).T

    q_vec = Matrix([phi_vec, theta_vec]).T.reshape(2*n_seg, 1)
    q_dot = Matrix([phi_dot_vec, theta_dot_vec]).T.reshape(2*n_seg, 1)
    segment_len = Matrix([length_vec[n] for n in range(n_seg)])
    seg_mass = Matrix([m_vec[n] for n in range(n_seg)])

    # OBS: The translation and rotation matrices are configured so the
    # segment is pointing in the direction of negative z-axis
    # Translation vector
    trans_vec, rotation_matrix =\
        get_modified_translation_and_rotation_matrices_z_up(theta_vec, phi_vec, n_seg,
                                                     segment_len, norm_len_s)

    # Transformation matrices including translation and rotation
    transformation_matrix = []
    for seg in range(n_seg):
        transf_matrix = rotation_matrix[3*seg:3 *
                                        (seg+1), :].row_join(trans_vec[seg, :].T)
        transf_matrix = transf_matrix.col_join(Matrix([[0, 0, 0, 1]]))
        transformation_matrix.append(transf_matrix)

    # create empty lists to store the Jacobian and mass matrices
    print("Calculating Jacobians and mass matrix ...")
    mass_matrices = []
    t_sum = zeros(3, 1)
    for i in range(n_seg):
        t_sum += trans_vec[i, :].T
        print(f"\tCalculating Jacobian for the segment {i} ...")
        jacobian_seg_i = t_sum.jacobian(q_vec)
        t_sum = t_sum.subs({norm_len_s: 1})
        # compute the mass matrix for the i-th segment
        print(f"\tCalculating mass matrix for the segment {i} ...")
        mass_seg_i = simplify((jacobian_seg_i.T) *
                              seg_mass[i] * jacobian_seg_i)
        print(f"\tIntegrating mass matrix for the segment {i} ...")
        mass_seg_is = simplify(integrate(mass_seg_i, (norm_len_s, 0, 1)))
        mass_matrices.append(mass_seg_is)

    print("Calculating the mass matrix ...")
    mass_matrix = zeros(2*n_seg, 2*n_seg)
    for i in range(n_seg):
        mass_matrix = mass_matrix + mass_matrices[i]

    print("Calculating the coriolis matrix ...")
    # Christoffel symbols
    ndim = len(q_vec)
    gamma_matrix = MutableDenseNDimArray.zeros(ndim, ndim, ndim)
    for i in range(ndim):
        for j in range(ndim):
            for k in range(ndim):
                func = 1/2 * (diff(mass_matrix[i, j], q_vec[k])
                              + diff(mass_matrix[i, k], q_vec[j])
                              - diff(mass_matrix[j, k], q_vec[i]))
                gamma_matrix[i, j, k] = func

    coriolis_matrix = zeros(ndim, ndim)
    for i in range(ndim):
        coriolis_matrix[i, :] = q_dot.T * Matrix(gamma_matrix[i, :, :])

    coriolis_matrix = simplify(coriolis_matrix)

    print("Calculating the gravity matrix ...")
    if n_seg > 1:
        ugi_matrices = []
        t_sum = zeros(3, 1)
        for i in range(n_seg):
            t_sum += trans_vec[i, :].T
            integrand_i = seg_mass[i] * grav * t_sum
            t_sum = t_sum.subs({norm_len_s: 1})
            ugi = integrate(integrand_i, (norm_len_s, 0, 1))
            ugi_matrices.append(ugi)
        gravitational_potential = zeros(1, 1)
        for i in range(n_seg):
            gravitational_potential = gravitational_potential + ugi_matrices[i]
        gravitational_potential = simplify(gravitational_potential)
    else:
        rotation_z = Matrix([[cos(phi_vec[0]), -sin(phi_vec[0]), 0],
                             [sin(phi_vec[0]), cos(phi_vec[0]), 0],
                             [0, 0, 1]])
        rotation_y = Matrix([[cos(theta_vec[0]/2), 0, sin(theta_vec[0]/2)],
                             [0, 1, 0],
                             [-sin(theta_vec[0]/2), 0, cos(theta_vec[0]/2)]])
        r_i = rotation_y *\
            Matrix([0, 0, segment_len/theta_vec[0] * sin(theta_vec[0]/2)])
        r_i = rotation_z * r_i
        r_0 = transformation_matrix[0] * Matrix([r_i, 1])

        integrand = seg_mass * grav * Matrix(r_0[:3, :])
        gravitational_potential = integrate(integrand, (norm_len_s, 0, 1))
    gravity_matrix = simplify(gravitational_potential.jacobian(q_vec).T)

    # translation vector and Jacobian used for kinematic control
    t_kin = []
    jac_kin = []
    rot_kin = [eye(3)] * n_seg
    for i in range(n_seg):
        if i == 0:
            t_kin_i = trans_vec[i, :].T
            jac_kin_i = t_kin_i.jacobian(q_vec)
        else:
            rot_kin[i] = rot_kin[i-1] * rotation_matrix[:3, :3].\
                subs(theta_vec[0], theta_vec[i-1]
                     ).subs(phi_vec[0], phi_vec[i-1]).subs(norm_len_s, 1)
            t_kin_i = t_kin[i-1].subs(norm_len_s, 1) +\
                rot_kin[i] * trans_vec[i, :].T
            jac_kin_i = t_kin_i.jacobian(q_vec)
        t_kin.append(t_kin_i)
        jac_kin.append(jac_kin_i)

    generated_sym_functions_folder = os.path.join(os.path.dirname(__file__),
                                                  "..", "..",
                                                  "generated_sym_functions")
    folder_to_save = os.path.join(generated_sym_functions_folder,
                                  f"{n_seg}_segments")
    fk_folder_to_save = os.path.join(generated_sym_functions_folder, "fk")
    print(f"Saving matrices to {folder_to_save} ...")
    os.makedirs(folder_to_save, exist_ok=True)
    os.makedirs(fk_folder_to_save, exist_ok=True)

    dill.settings['recurse'] = True
    phi, theta, segment_len = symbols('phi theta L')

    print("Lambdifying Jacobian matrix ...")
    jac_rot_lamb = lambdify((*phi_vec, *theta_vec, *length_vec, norm_len_s),
                            jac_kin[-1], modules='numpy')
    jac_rot_lamb.__doc__ =\
        ("Returns the Jacobian matrix\nArgs:\n"
         "\tphi[0...n] (float): Orientation angles in"
         " radians of each segment\n"
         "\ttheta_vec[0...n] (float): Inclination (bending)"
         " angles in radians of each segment\n"
         "\tL[0...n] (float): lengths of each segments in meters\n"
         "\ts (float): Position along the segment normalized [0,1]\n"
         "Returns:\n"
         "\tJ (np.array): Jacobian matrix of shape (3, 2)\n")

    # Saving .dill file
    jac_name = f"j{n_seg}_rot.dill"
    jac_folder_to_save = os.path.join(folder_to_save, jac_name)
    print(f"Saving {jac_name} ...")
    with open(jac_folder_to_save, 'wb') as file:
        dill.dump(jac_rot_lamb, file)

    # Saving .txt file
    jac_name = f"j{n_seg}_rot.txt"
    jac_func_txt_path = os.path.join(folder_to_save, jac_name)
    print(f"Saving {jac_name} ...")
    with open(jac_func_txt_path, "wb") as outf:
        outf.write(str(jac_kin[-1]).encode())

    print("Lambdifying fk_rot matrix ...")
    fk_rot_lamb = lambdify((*phi_vec, *theta_vec, *length_vec, norm_len_s),
                           t_kin[-1], modules='numpy')
    fk_rot_lamb.__doc__ =\
        ("Returns the forward kinematics\nArgs:\n"
         "\tphi[0...n] (float): Orientation angles in"
         " radians of each segment\n"
         "\ttheta_vec[0...n] (float): Inclination (bending)"
         " angles in radians of each segment\n"
         "\tL[0...n] (float): lengths of each segments in meters\n"
         "\ts (float): Position along the segment normalized [0,1]\n")

    # Saving .dill file
    fk_name = f"fk{n_seg}_rot.dill"
    fk_folder_path = os.path.join(folder_to_save, fk_name)
    print(f"Saving {fk_name} ...")
    with open(fk_folder_path, 'wb') as file:
        dill.dump(fk_rot_lamb, file)

    # Saving .txt file
    fk_name = f"fk{n_seg}_rot.txt"
    fk_func_txt_path = os.path.join(folder_to_save, fk_name)
    print(f"Saving {fk_name} ...")
    with open(fk_func_txt_path, "wb") as outf:
        outf.write(str(t_kin[-1]).encode())

    print("Lambdifying the translation vector ...")
    # The t translation vector can be used by any segment link in general
    # It is mainly used to plot the segments
    translation_vec = trans_vec[0, :].subs(
        theta_vec[0], theta).subs(phi_vec[0], phi)
    translation_vec = translation_vec.subs(
        length_vec[0], segment_len).reshape(3, 1)
    fk_func = lambdify((phi, theta, segment_len, norm_len_s),
                       translation_vec, modules='numpy')
    fk_func.__doc__ =\
        ("Returns the translation vector\nArgs:\n"
         "\tphi (float): Orientation angle in radians\n"
         "\ttheta (float): Inclination (bending) angle in radians\n"
         "\tL (float): Length of the segment\n"
         "\ts (float): Position along the segment normalized [0,1]\n"
         "Returns:\n"
         "\ttranslation_vector (np.array): Translation vector of the "
         " segment\n"
         "Example:\n"
         "\t>>> translation_vector = fk_func(phi, theta, L, ""s)\n")

    print("Lambdifying the rotation matrix ...")
    # The R_new matrix can be used by any segment link in general
    # It is mainly used to plot the segments
    rot = rotation_matrix[:3, :3].subs(theta_vec[0], theta).subs(
        phi_vec[0], phi).subs(length_vec[0], segment_len)
    rot_func = lambdify((phi, theta, norm_len_s), rot, modules='numpy')
    rot_func.__doc__ =\
        ("Returns the rotation matrix\nArgs:\n"
         "\tphi (float): Orientation angle in radians\n"
         "\ttheta (float): Inclination (bending) angle in radians\n"
         "\ts (float): Position along the segment normalized [0,1]\n"
         "Returns:\n"
         "\trotation_matrix (np.array): Rotation matrix of the"
         "segment\n"
         "Example:\n"
         "\t>>> rotation_matrix = rot_func(phi, theta, s)\n")

    print("Lambdifying G matrix ...")
    grav_func = lambdify((*phi_vec, *theta_vec, *length_vec, *m_vec, grav_x, grav_y,
                          grav_z), gravity_matrix, modules='numpy')
    grav_func.__doc__ =\
        ("Returns the gravity matrix\nArgs:\n"
         "\tphi[0...n] (float): Orientation angles in"
         "radians of each segment\n"
         "\ttheta_vec[0...n] (float): Inclination (bending)"
         " angles in radians of each segment\n"
         "\tL[0...n] (float): lengths of each segments in meters\n"
         "\tm[0...n] (float): Each segment's mass in kg\n"
         "\tgx (float): gravity in the x direction\n"
         "\tgy (float): gravity in the y direction\n"
         "\tgz (float): gravity in the z direction\n"
         "Returns:\n"
         "\tG: gravity matrix [2*num_segments, 1]\n"
         "Example:\n"
         "\t>>> import dill, os \n"
         "\t>>> G_dill = dill.load(open(os.path.join("
         "folder_to_load, 'M.dill'), 'rb'))\n"
         "\t>>> # Code for 2 segments\n"
         "\t>>> phi_vec = [1.57079633, 1.57079633]\n"
         "\t>>> theta_vec = [1.57079633, 1.57079633]\n"
         "\t>>> length_vec = [0.25, 0.25]\n"
         "\t>>> m_vec = [0.5, 0.5]\n"
         "\t>>> gx = 0.0\n"
         "\t>>> gy = 0.0\n"
         "\t>>> gz = -9.81\n"
         "\t>>> G_dill(*phi_vec, *theta_vec, *length_vec,"
         " *m_vec, gx,"
         "gy, gz)\n")

    print("Lambdifying M matrix ...")
    mass_func = lambdify(
        (*phi_vec, *theta_vec, *length_vec, *m_vec), mass_matrix, 'numpy')
    mass_func.__doc__ =\
        ("Returns the mass matrix\nArgs:\n"
         "\tphi[0...n] (float): Orientation angles in"
         "radians of each segment\n"
         "\ttheta_vec[0...n] (float): Inclination (bending)"
         " angles in radians of each segment\n"
         "\tL[0...n] (float): lengths of each segments in meters\n"
         "\tm[0...n] (float): Each segment's mass in kg\n"
         "Returns:\n"
         "\tM (array): Mass matrix [2*n_segments, 2*n_segments]\n"
         "Example:\n"
         "\t>>> import dill, os \n"
         "\t>>> M_dill = dill.load(open(os.path.join("
         " folder_to_load, 'M.dill'), 'rb'))\n"
         "\t>>> # Code for 2 segments\n"
         "\t>>> phi_vec = [1.57079633, 1.57079633]\n"
         "\t>>> theta_vec = [1.57079633, 1.57079633]\n"
         "\t>>> length_vec = [0.25, 0.25]\n"
         "\t>>> m_vec = [0.5, 0.5]\n"
         "\t>>> M_dill(*phi_vec, *theta_vec, *length_vec, *m_vec)\n")

    print("Lambdifying C matrix ...")
    coriolis_func = lambdify((*phi_vec, *theta_vec, *phi_dot_vec,
                              *theta_dot_vec, *length_vec, *m_vec),
                             coriolis_matrix, 'numpy')
    coriolis_func.__doc__ =\
        ("Returns the coriolis matrix\nArgs:\n"
         "\tphi_dot [0...n] (float): Orientation angles"
         " velocity in radians/s of each segment\n"
         "\ttheta_vec[0...n] (float): Inclination (bending)"
         " angles in radians of each segment\n"
         "\tL[0...n] (float): lengths of each segments in"
         " meters\n"
         "\tm[0...n] (float): Each segment's mass in kg\n"
         "Returns:\n"
         "\tC (array): Mass matrix [2*n_seg, 2*n_seg]\n"
         "Example:\n"
         "\t>>> import dill, os \n"
         "\t>>> C_dill = dill.load(open(os.path.join("
         " folder_to_load, 'C.dill'), 'rb'))\n"
         "\t>>> # Code for 2 segments\n"
         "\t>>> phi_vec = [1.57079633, 1.57079633]\n"
         "\t>>> theta_vec = [1.57079633, 1.57079633]\n"
         "\t>>> length_vec = [0.25, 0.25]\n"
         "\t>>> m_vec = [0.5, 0.5]\n"
         "\t>>> C_dill(*phi_vec, *theta_vec, *phi_dot_vec,"
         "*theta_dot_vec, *length_vec, *m_vec)\n")

    grav_func_path = os.path.join(folder_to_save, f"G{n_seg}.dill")
    m_func_path = os.path.join(folder_to_save, f"M{n_seg}.dill")
    c_func_path = os.path.join(folder_to_save, f"C{n_seg}.dill")
    rot_func_path = os.path.join(fk_folder_to_save, "rot_func.dill")
    fk_func_path = os.path.join(fk_folder_to_save, "fk_func.dill")
    with open(grav_func_path, "wb") as file:
        dill.dump(grav_func, file)
    with open(m_func_path, "wb") as file:
        dill.dump(mass_func, file)
    with open(c_func_path, "wb") as file:
        dill.dump(coriolis_func, file)
    with open(fk_func_path, "wb") as file:
        dill.dump(fk_func, file)
    with open(rot_func_path, "wb") as file:
        dill.dump(rot_func, file)

    print("Saving the matrices in a human-readable format ...")
    grav_func_txt_path = os.path.join(folder_to_save, f"G{n_seg}.txt")
    with open(grav_func_txt_path, "wb") as outf:
        outf.write(str(gravity_matrix).encode())
    m_func_txt_path = os.path.join(folder_to_save, f"M{n_seg}.txt")
    with open(m_func_txt_path, "wb") as outf:
        outf.write(str(mass_matrix).encode())
    c_func_txt_path = os.path.join(folder_to_save, f"C{n_seg}.txt")
    with open(c_func_txt_path, "wb") as outf:
        outf.write(str(coriolis_matrix).encode())
    fk_func_txt_path = os.path.join(fk_folder_to_save, "fk_func.txt")
    with open(fk_func_txt_path, "wb") as outf:
        outf.write(str(trans_vec).encode())
    rot_func_txt_path = os.path.join(fk_folder_to_save, "rot_func.txt")
    with open(rot_func_txt_path, "wb") as outf:
        outf.write(str(trans_vec).encode())


if __name__ == "__main__":
    DESCRIPTION = 'Generate G, M and C matrices for n segment links.'
    parser = argparse.ArgumentParser(DESCRIPTION)
    parser.add_argument('--links', dest='n_segment_links', default=1,
                        required=False, type=int,
                        help=('Number of segment links to generate G, M'
                              'and C matrices.'))
    args = parser.parse_args()
    generate_n_segment_symbolic_functions(args.n_segment_links)
