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

import matplotlib.pyplot as plt
import numpy as np


class PlotSegmentGeneral():
    """Class to plot multiple curves of soft robots."""

    def __init__(self):
        self.curves_to_plot = []
        self.fig = None
        self.ax = None

    def add_curve(
        self,
        curve_name: str,
        curve_data: np.array
    ):
        """
        Add curve to plot.

        Parameters
        ----------
        curve_name : str
            Name of the curve
        curve_data : np.array
            array of shape (3, n) where n is the number of points of the curve

        """
        self.curves_to_plot.append({
            "curve_name": curve_name,
            "curve_data": curve_data,
            "line_qs": []
        })

    def view(self):
        """
        Plot segments.

        Parameters
        ----------
        state: np.array
            State vector (phi0, theta0, phi1, theta1, ..., phin, thetan)
        frames_position_isaac_sim: list
            List of frames position from Isaac Sim.

        """
        # Activate interactive figure
        plt.ion()
        # creating subplot and figure
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(projection='3d')

        for curve_dict in self.curves_to_plot:
            curve_dict['line_qs'].append(
                self.ax.plot(
                    curve_dict['curve_data'][0, :],
                    curve_dict['curve_data'][1, :],
                    curve_dict['curve_data'][2, :],
                    label=curve_dict['curve_name'])
            )

        quiverl = 0.05
        self.ax.quiver(0, 0, 0, quiverl, 0, 0, color='r')
        self.ax.quiver(0, 0, 0, 0, quiverl, 0, color='g')
        self.ax.quiver(0, 0, 0, 0, 0, quiverl, color='b')
        self.ax.set_xlim(-0.25, 0.25)
        self.ax.set_ylim(-0.25, 0.25)
        self.ax.set_zlim(-0.25, 0.25)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        plt.grid()
        plt.legend()

    def update_curve_data(
        self,
        curve_name: str,
        curve_data: np.array
    ):
        """
        Update curve data.

        Parameters
        ----------
        curve_name : str
            Name of the curve
        curve_data : np.array
            array of shape (3, n) where n is the number of points of the curve

        """
        for curve in self.curves_to_plot:
            if curve['curve_name'] == curve_name:
                curve['curve_data'] = curve_data

    def update_plot(
        self
    ) -> None:
        """Update plot with new state."""
        for curve in self.curves_to_plot:
            curve['line_qs'][0][0].set_data_3d(
                curve['curve_data'][0, :],
                curve['curve_data'][1, :],
                curve['curve_data'][2, :])

        # re-drawing the figure
        self.fig.canvas.draw()

        # to flush the GUI events
        self.fig.canvas.flush_events()

    def get_sphere(
        self,
        goal: np.array
    ):
        """
        Get sphere to plot goal position.

        Parameters
        ----------
        goal: np.array
            Goal position.

        """
        radius = 0.005
        u_ver, v_ver = np.mgrid[0:2*np.pi:50j, 0:2*np.pi:50j]
        x_pos = np.cos(u_ver)*np.sin(v_ver)*radius + goal[0]
        y_pos = np.sin(u_ver)*np.sin(v_ver)*radius + goal[1]
        z_pos = np.cos(v_ver)*radius + goal[2]
        return x_pos, y_pos, z_pos
