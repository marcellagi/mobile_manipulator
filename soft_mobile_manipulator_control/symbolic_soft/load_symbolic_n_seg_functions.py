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
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
# flake8 test complains about the import below, but it is necessary to load
# the symbolic functions
if 'TESTING' not in os.environ:
    from helper_funcs.load_sym_functions import LoadSymbolic


def load_symbolic_n_seg_functions(num_segments):
    """
    Load and test values from the generated symbolic equations.

    Parameters
    ----------
    num_segments : int
        Number of segments to load symbolic equations.

    """
    condition = isinstance(num_segments, int) and num_segments > 0
    assert condition, "num_segments must be a positive integer"

    dir_name = os.path.dirname
    path = dir_name(dir_name(dir_name(os.path.realpath(__file__))))
    symbolic_funcs = LoadSymbolic(num_segments, path)

    print("Testing lambdified functions loaded from .dill files...")
    phi_vec = [1.57079633] * num_segments
    theta_vec = [1.57079633] * num_segments
    phi_dot_vec = [1] * num_segments
    theta_dot_vec = [1] * num_segments
    m_vec = [0.5] * num_segments
    length_vector = [0.25] * num_segments
    norm_seg_len = 1

    print("G matrix function: \n" +
          symbolic_funcs.dynamic_matrices.grav_matrix.__doc__)
    print("M matrix function: \n" +
          symbolic_funcs.dynamic_matrices.mass_matrix.__doc__)
    print("C matrix function: \n" +
          symbolic_funcs.dynamic_matrices.coriolis_matrix.__doc__)
    print("FK function: \n" +
          symbolic_funcs.kinematic_matrices.fk_dill.__doc__)
    print("Rot function: \n" +
          symbolic_funcs.kinematic_matrices.rot_dill.__doc__)

    gravity_matrix = symbolic_funcs.dynamic_matrices.grav_matrix(
        *phi_vec, *theta_vec, *length_vector, *m_vec, 0, 0, -9.81)
    mass_matrix = symbolic_funcs.dynamic_matrices.mass_matrix(
        *phi_vec, *theta_vec, *length_vector, *m_vec)
    coriolis_matrix =\
        symbolic_funcs.dynamic_matrices.coriolis_matrix(*phi_vec, *theta_vec,
                                                        *phi_dot_vec,
                                                        *theta_dot_vec,
                                                        *length_vector,
                                                        *m_vec)
    fk_func = symbolic_funcs.kinematic_matrices.fk_dill(
        phi_vec[0], theta_vec[0], length_vector[0], norm_seg_len)
    rot_func = symbolic_funcs.kinematic_matrices.rot_dill(
        phi_vec[0], theta_vec[0], norm_seg_len)
    print(f"G matrix {gravity_matrix.shape}:\n {gravity_matrix}")
    print(f"M matrix {mass_matrix.shape}:\n {mass_matrix}")
    print(f"C matrix {coriolis_matrix.shape}:\n {coriolis_matrix}")
    print(f"FK function {fk_func.shape}:\n {fk_func}")
    print(f"Rot function {rot_func.shape}:\n {rot_func}")


if __name__ == "__main__":
    DESCRIPTION = 'Generate G, M and C matrices for n segment links.'
    parser = argparse.ArgumentParser(DESCRIPTION)
    parser.add_argument('--links', dest='n_segment_links', default=3,
                        required=False, type=int,
                        help='Number of segment links to load G, M and'
                        ' C matrices.')
    args = parser.parse_args()
    # Load lambdified functions
    load_symbolic_n_seg_functions(args.n_segment_links)
