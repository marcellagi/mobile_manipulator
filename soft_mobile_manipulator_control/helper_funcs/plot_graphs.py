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

import matplotlib.pyplot as plt
import numpy as np


class SegmentGraph():
    """Plot graphs for position, velocity and acceleration of each segment."""

    def __init__(self, seg_param, n_links, simulation_time, dt_time):
        """
        Initialize parameters to plot position, velocity and acc graphs.

        Parameters
        ----------
        seg_param : dict
            Segment parameters.
        n_links : int
            Number of links.
        simulation_time : float
            Simulation time.
        dt_time : float
            Time step.

        """
        self.seg_param = seg_param
        self.n_links = n_links

        self.position = np.array(self.seg_param['state'][0:self.n_links*2])
        self.ref_position = np.array(
            self.seg_param['ref_state'][0:self.n_links*2])
        self.velocity = np.array(
            self.seg_param['state'][self.n_links*2:self.n_links*4])
        self.acceleration = np.array(
            self.seg_param['state'][self.n_links*4:self.n_links*6])
        self.torque = np.zeros((1, self.n_links*2))

        self.dt_list = np.arange(0, simulation_time+dt_time, dt_time)

    def update_plot(self, seg_param: dict, tau: np.array) -> None:
        """
        Update state variables to plot.

        Parameters
        ----------
        seg_param: dict
            segment parameters.
        tau: np.array
            Torque of each segment.

        """
        self.position = np.vstack(
            (self.position, seg_param['state'][0:self.n_links*2]))
        self.ref_position = np.vstack(
            (self.ref_position, seg_param['ref_state'][0:self.n_links*2]))
        self.velocity = np.vstack(
            (self.velocity,
             self.seg_param['state'][self.n_links*2:self.n_links*4]))
        self.acceleration = np.vstack(
            (self.acceleration,
             self.seg_param['state'][self.n_links*4:self.n_links*6]))
        self.torque = np.vstack((self.torque, tau.T))

    def plot_graphs(self) -> None:
        """Plot position, velocity and acceleration graphs."""
        colors_phi = ["blue", "black"]
        colors_theta = ["brown", "cyan"]
        plt.figure(figsize=(10, 10))
        plt.suptitle("Position (deg)", y=0.92)
        seg_i = 0
        for i in range(0, self.n_links*2, 2):
            plt.subplot(self.n_links, 1, seg_i+1)
            plt.plot(self.dt_list, np.rad2deg(
                self.position[:, i]), color=colors_phi[0],
                linestyle='dashed', label=r'$\phi_{'+str(seg_i)+'}$')
            plt.plot(self.dt_list, np.rad2deg(
                self.position[:, i+1]), color=colors_theta[0],
                linestyle='dashed', label=r'$\theta_{'+str(seg_i)+'}$')
            plt.plot(self.dt_list, np.rad2deg(
                self.ref_position[:, i]), color=colors_phi[1],
                label=r'$\phi_{ref'+str(seg_i)+'}$')
            plt.plot(self.dt_list, np.rad2deg(
                self.ref_position[:, i+1]), color=colors_theta[1],
                label=r'$\theta_{ref'+str(seg_i)+'}$')
            plt.legend(loc="lower right", ncol=self.n_links*2)
            plt.ylabel(f"q{seg_i} (deg)")
            seg_i += 1
        plt.xlabel("Time (s)")

        plt.figure(figsize=(10, 10))
        seg_i = 0
        plt.suptitle("Velocity (deg/s)", y=0.92)
        for i in range(0, self.n_links*2, 2):
            plt.subplot(self.n_links, 1, seg_i+1)
            plt.plot(self.dt_list, np.rad2deg(
                self.velocity[:, i]), color=colors_phi[0],
                label=r'$\dot{\phi}_{'+str(seg_i)+'}$')
            plt.plot(self.dt_list, np.rad2deg(
                self.velocity[:, i+1]), color=colors_theta[0],
                label=r'$\dot{\theta}_{'+str(seg_i)+'}$')
            plt.legend(loc="lower right", ncol=self.n_links*2)
            plt.ylabel(f"qdot{seg_i} (deg)")
            seg_i += 1
        plt.xlabel("Time (s)")

        plt.figure(figsize=(10, 10))
        seg_i = 0
        plt.suptitle("Acceleration (deg/s^2)", y=0.92)
        for i in range(0, self.n_links*2, 2):
            plt.subplot(self.n_links, 1, seg_i+1)
            plt.plot(self.dt_list, np.rad2deg(
                self.acceleration[:, i]), color=colors_phi[0],
                label=r'$\ddot{\phi}_{'+str(seg_i)+'}$')
            plt.plot(self.dt_list, np.rad2deg(
                self.acceleration[:, i+1]), color=colors_theta[0],
                label=r'$\ddot{\theta}_{'+str(seg_i)+'}$')
            plt.legend(loc="lower right", ncol=self.n_links*2)
            plt.ylabel(f"qddot{seg_i} (deg)")
            seg_i += 1
        plt.xlabel("Time (s)")

        plt.figure(figsize=(10, 10))
        seg_i = 0
        self.torque = np.radians(self.torque)
        plt.suptitle("Torque", y=0.92)
        for i in range(0, self.n_links*2, 2):
            plt.subplot(self.n_links, 1, seg_i+1)
            plt.plot(self.dt_list, np.rad2deg(
                self.torque[:, i]), color=colors_phi[0],
                label=r'$\tau{\phi}_{'+str(seg_i)+'}$')
            plt.plot(self.dt_list, np.rad2deg(
                self.torque[:, i+1]), color=colors_theta[0],
                label=r'$\tau{\theta}_{'+str(seg_i)+'}$')
            plt.legend(loc="lower right", ncol=self.n_links*2)
            plt.ylabel(f"Torque{seg_i}")
            seg_i += 1
        plt.xlabel("Time (s)")

        plt.show(block=True)
