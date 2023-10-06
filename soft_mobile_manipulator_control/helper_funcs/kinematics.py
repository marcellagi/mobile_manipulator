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

import numpy as np


def rotation_matrix_x(angle):
    """
    Rotation matrix around the x axis.

    Parameters
    ----------
    angle: float
        Rotation

    Returns
    -------
    np.array
        Rotation matrix

    """
    return np.array([[1, 0, 0],
                     [0, np.cos(angle), -np.sin(angle)],
                     [0, np.sin(angle), np.cos(angle)]])


def rotation_matrix_y(angle):
    """
    Rotation matrix around the y axis.

    Parameters
    ----------
    angle: float
        Rotation angle in radians

    Returns
    -------
    np.array
        Rotation matrix

    """
    return np.array([[np.cos(angle), 0, np.sin(angle)],
                     [0, 1, 0],
                     [-np.sin(angle), 0, np.cos(angle)]])


def rotation_matrix_z(angle):
    """
    Rotation matrix around the z axis.

    Parameters
    ----------
    angle: float
        Rotation angle in radians

    Returns
    -------
    np.array
        Rotation matrix

    """
    return np.array([[np.cos(angle), -np.sin(angle), 0],
                     [np.sin(angle), np.cos(angle), 0],
                     [0, 0, 1]])


def transformation_matrix_from_rotation(rotation_matrix, translation_vector):
    """
    Create transform matrix from rotation matrix and translation vector.

    Parameters
    ----------
    rotation_matrix: np.array
        Rotation matrix
    translation_vector: np.array
        Translation vector

    Returns
    -------
    np.array
        Homogeneous transformation matrix

    """
    transformation_matrix = np.hstack(
        (rotation_matrix, translation_vector))
    transformation_matrix = np.vstack(
        (transformation_matrix, np.array([0, 0, 0, 1])))
    return transformation_matrix


def htm_rotation_around_x(angle) -> np.array:
    """
    Get the rotation matrix around x axis.

    Parameters
    ----------
    angle : float
        Angle in radians.

    Returns
    -------
    np.array
        Rotation matrix

    """
    return np.array([
        [1, 0, 0, 0],
        [0, np.cos(angle), -np.sin(angle), 0],
        [0, np.sin(angle), np.cos(angle), 0],
        [0, 0, 0, 1]
    ])


def htm_rotation_around_y(angle) -> np.array:
    """
    Rotation matrix around y axis.

    Parameters
    ----------
    angle : float
        Angle in radians.

    Returns
    -------
    np.array
        Rotation matrix around z axis

    """
    return np.array([
        [np.cos(angle), 0, np.sin(angle), 0],
        [0, 1, 0, 0],
        [-np.sin(angle), 0, np.cos(angle), 0],
        [0, 0, 0, 1]
    ])


def htm_rotation_around_z(angle: float) -> np.array:
    """
    Rotation matrix around z axis.

    Parameters
    ----------
    angle : float
        Angle in radians.

    Returns
    -------
    np.array
        Rotation matrix around z axis

    """
    return np.array([
        [np.cos(angle), -np.sin(angle), 0, 0],
        [np.sin(angle), np.cos(angle), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])


def translation_matrix(
    x_coord: float,
    y_coord: float,
    z_coord: float
) -> np.array:
    """
    Generate the translation matrix.

    Parameters
    ----------
    x_coord : float
        Translation in x axis.
    y_coord : float
        Translation in y axis.
    z_coord : float
        Translation in z axis.

    Returns
    -------
    np.array
        Translation matrix.

    """
    return np.array([
        [1, 0, 0, x_coord],
        [0, 1, 0, y_coord],
        [0, 0, 1, z_coord],
        [0, 0, 0, 1]
    ])


def change_htm_translation(
    htm: np.array,
    translation: np.array
) -> np.array:
    """
    Build homogeneous transformation matrix.

    Parameters
    ----------
    htm : np.array
        Homogeneous transformation matrix.
    translation : np.array
        Translation vector.

    Returns
    -------
    np.array
        Homogeneous transformation matrix.

    """
    htm[0, -1] = translation[0]
    htm[1, -1] = translation[1]
    htm[2, -1] = translation[2]
    return htm
