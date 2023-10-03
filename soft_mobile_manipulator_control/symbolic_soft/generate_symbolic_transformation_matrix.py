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

import os
from sympy import Matrix, cos, simplify, sin, symbols


def rotation_matrix_z(phi):
    """
    Rotation matrix about z axis.

    Parameters
    ----------
    phi : float
        Rotation angle about z axis.

    Returns
    -------
    Matrix
        Rotation matrix about z axis.

    """
    return Matrix([[cos(phi), -sin(phi), 0],
                   [sin(phi), cos(phi), 0],
                   [0, 0, 1]])


def rotation_matrix_y(theta):
    """
    Rotation matrix about y axis.

    Parameters
    ----------
    theta : float
        Rotation angle about y axis.

    Returns
    -------
    Matrix
        Rotation matrix about y axis.

    """
    return Matrix([[cos(theta), 0, sin(theta)],
                   [0, 1, 0],
                   [-sin(theta), 0, cos(theta)]])


def create_transformation_matrix(rotation_matrix, translation_vector):
    """
    Create transformation matrix from rotation matrix and translation vector.

    Parameters
    ----------
    rotation_matrix : Matrix
        Rotation matrix.
    translation_vector : Matrix
        Translation vector.

    Returns
    -------
    Matrix
        Homogeneous transformation matrix.

    """
    return rotation_matrix.row_join(
        translation_vector).col_join(Matrix([[0, 0, 0, 1]]))


def generate_symbolic_rotation_matrix():
    """Generate symbolic homogeneous transformation matrix."""
    phi, theta, radius = symbols('phi theta r')

    # Rotation about z axis + pi (offset around z axis)
    # This rotation is needed to align the segment plane with the x-z axis in
    # the -x direction
    r_z_plus_offset = rotation_matrix_z(phi)
    t_z_plus_offset = create_transformation_matrix(r_z_plus_offset,
                                                   Matrix([[0, 0, 0]]).T)

    # Rotation about z axis - pi (offset around z axis)
    r_z_minus_offset = rotation_matrix_z(-phi)
    t_z_minus_offset = create_transformation_matrix(r_z_minus_offset,
                                                    Matrix([[0, 0, 0]]).T)

    # Rotation about y axis
    r_y = rotation_matrix_y(theta)
    # t_xz = Matrix([radius*(cos(theta) - 1), 0, -radius*sin(theta)])
    t_xz = Matrix([radius*(1 - cos(theta)), 0, radius*sin(theta)])
    t_ry_txz = r_y.row_join(t_xz).col_join(Matrix([[0, 0, 0, 1]]))

    # The expression sin(phi)**2 + cos(phi)**2*cos(theta) in the position [0,0]
    # and [1, 1] in the matrix T is the same as cos(phi)**2*(cos(theta)-1) + 1
    # after applying some trigonometric identities.
    # Therefore, the transformation matrix below is the same as found in the
    # Kyle`s code and in the paper "Kinematics for Multisection Continuum
    # Robots" by Bryan A. Jones and Ian D. Walker (2006)
    transfom_matrix = simplify(t_z_plus_offset * t_ry_txz * t_z_minus_offset)

    transl_vector = os.path.join(os.path.dirname(__file__),
                                 "transl_vector.txt")
    with open(transl_vector, "wb") as outf:
        outf.write(str(transfom_matrix[:, -1]).encode())

    rot_matrix = os.path.join(os.path.dirname(__file__),
                              "rot_matrix.txt")
    with open(rot_matrix, "wb") as outf:
        outf.write(str(transfom_matrix[:3, :3]).encode())


if __name__ == '__main__':
    generate_symbolic_rotation_matrix()
