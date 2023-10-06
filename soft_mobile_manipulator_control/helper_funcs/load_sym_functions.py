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
import dataclasses

import numpy as np
import dill


@dataclasses.dataclass
class DynamicMatrices():
    """
    Class to store the dynamic matrices of the soft robot.

    Parameters
    ----------
    grav_matrix: np.ndarray
        Gravity matrix.
    mass_matrix: np.ndarray
        Mass matrix.
    coriolis_matrix: np.ndarray
        Coriolis matrix.

    """

    grav_matrix: np.ndarray
    mass_matrix: np.ndarray
    coriolis_matrix: np.ndarray


@dataclasses.dataclass
class KinematicMatrices():
    """
    Class to store the kinematic matrices of the soft robot.

    Parameters
    ----------
    fk_dill: np.ndarray:
        Forward kinematics.
    rot_dill: np.ndarray
        Rotation matrix.
    fk_rot: np.ndarray
        Forward kinematics rotation matrix dill file.
    jac_rot: np.ndarray
        Jacobian rotation matrix.

    """

    fk_dill: np.ndarray
    rot_dill: np.ndarray
    fk_rot: np.ndarray
    jac_rot: np.ndarray


class LoadSymbolic():
    """Class to load symbolic functions from dill files."""

    def __init__(self, n_links, path=None):
        """
        Load symbolic functions from dill files.

        Parameters
        ----------
        n_links: int
            Number of segments of the soft robot.
        path: str
            Path to the dill files.

        """
        self.n_links = n_links

        if path is None:
            # NOTE: We have to import this package here because this script is
            # called using ROS or common python script. If we import this package
            # using python script, the package will not be found.
            from ament_index_python.packages import get_package_share_directory
            self.pkg_path = get_package_share_directory('softrobots_dynamic_model')
        else:
            self.pkg_path = path

        # Open lambdified functions
        self.sym_seg_path =\
            os.path.join(self.pkg_path, "generated_sym_functions",
                         f"{self.n_links}_segments")
        self.sym_kin_path =\
            os.path.join(self.pkg_path, "generated_sym_functions", "fk")

        self.verify_symbolic_matrices_existence()

        # Gravity matrix
        grav_matrix = self.load_gravity_matrix()

        # Mass matrix
        mass_matrix = self.load_mass_matrix()

        # Coriolis matrix
        coriolis_matrix = self.load_coriolis_matrix()

        self.dynamic_matrices = DynamicMatrices(grav_matrix,
                                                mass_matrix,
                                                coriolis_matrix)

        # Forward kinematics
        fk_dill = self.load_forward_kinematics()

        # Rotation matrix
        rot_dill = self.load_rotation_matrix()

        # Forward kinematics rotation matrix
        fk_rot = self.load_fk_rot_matrix()

        # Jacobian of rotational matrix
        jac_rot = self.load_jacobian_rotational_matrix()

        # Kinematic matrices
        self.kinematic_matrices = KinematicMatrices(fk_dill, rot_dill,
                                                    fk_rot, jac_rot)

    def verify_symbolic_matrices_existence(self):
        """Verify if the dill files exist."""
        # Verify if the dill files exist
        if not os.path.exists(self.sym_seg_path):
            raise FileNotFoundError("The symbolic matrices were not found. "
                                    "Please, run the script "
                                    "'generate_symbolic_matrices.py' "
                                    "to generate the symbolic matrices.")

    def load_forward_kinematics(self):
        """Load forward kinematics function from dill file."""
        fk_func_path = os.path.join(self.sym_kin_path, "fk_func.dill")
        with open(fk_func_path, "rb") as file:
            fk_dill = dill.load(file)
        return fk_dill

    def load_rotation_matrix(self):
        """Load rotation matrix function from dill file."""
        rot_path = os.path.join(self.sym_kin_path, "rot_func.dill")
        with open(rot_path, "rb") as file:
            rot_dill = dill.load(file)
        return rot_dill

    def load_gravity_matrix(self):
        """Load gravity matrix function from dill file."""
        gravity_function_path = os.path.join(self.sym_seg_path,
                                             f"G{self.n_links}.dill")
        with open(gravity_function_path, "rb") as file:
            grav_matrix = dill.load(file, "rb")
        return grav_matrix

    def load_mass_matrix(self):
        """Load mass matrix function from dill file."""
        mass_matrix_path = os.path.join(self.sym_seg_path,
                                        f"M{self.n_links}.dill")
        with open(mass_matrix_path, "rb") as file:
            mass_matrix = dill.load(file, "rb")
        return mass_matrix

    def load_coriolis_matrix(self):
        """Load coriolis matrix function from dill file."""
        coriolis_matrix_path = os.path.join(self.sym_seg_path,
                                            f"C{self.n_links}.dill")
        with open(coriolis_matrix_path, "rb") as file:
            coriolis_matrix = dill.load(file, "rb")
        return coriolis_matrix

    def load_fk_rot_matrix(self):
        """Load forward kinematics rotation matrix function from dill file."""
        fk_rot_path = os.path.join(self.sym_seg_path, f"fk{self.n_links}_rot.dill")
        with open(fk_rot_path, "rb") as file:
            fk_rot = dill.load(file)
        return fk_rot

    def load_jacobian_rotational_matrix(self):
        """Load jacobian of rotational matrix function from dill file."""
        jac_rot_path = os.path.join(self.sym_seg_path,
                                    f"j{self.n_links}_rot.dill")
        with open(jac_rot_path, "rb") as file:
            jac_rot = dill.load(file, "rb")
        return jac_rot
