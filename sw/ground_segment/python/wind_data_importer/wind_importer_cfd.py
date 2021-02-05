#!/usr/bin/env python

"""/*
 * Copyright (C) Sunyou Hwang <S.Hwang-1@tudelft.nl>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
"""

import numpy as np
from scipy.spatial import KDTree
import time
# from scipy import interpolate


class CFDImporter:
    def __init__(self):
        self.cfd_points = None
        self.cfd_u = None
        self.interpolator_x = None
        self.interpolator_y = None
        self.interpolator_z = None
        self.translate_origin = (0, 0, 0)
        self.wind_data = None
        self.count = 0
        self.x_min, self.x_max, self.y_min, self.y_max, self.z_min, self.z_max = 0, 0, 0, 0, 0, 0
        self.inlet_wind_speed_enu = (0, 0, 0)
        self.inlet_wind_speed_sum = 0
        self.kd_tree = None
        self.count = 0

    def init_importer(self, f_name, w_e, w_n, w_u):
        print("init importer")
        self.read_csv(f_name)
        self.x_min = np.min(self.wind_data[:, 0])
        self.x_max = np.max(self.wind_data[:, 0])
        self.y_min = np.min(self.wind_data[:, 1])
        self.y_max = np.max(self.wind_data[:, 1])
        self.z_min = np.min(self.wind_data[:, 2])
        self.z_max = np.max(self.wind_data[:, 2])
        self.inlet_wind_speed_enu = (w_e, w_n, w_u)
        self.inlet_wind_speed_sum = abs(w_e)+abs(w_n)+abs(w_u)
        print("x: ", self.x_min, self.x_max, ", y: ", self.y_min, self.y_max, ", z: ", self.z_min, self.z_max)

    def translate_field(self, x=0, y=0, z=0):
        self.translate_origin += (x, y, z)

    def read_csv(self, f_name):
        csv_data = np.genfromtxt(fname=f_name, delimiter=",", skip_header=6)
        _idx = np.where(csv_data[:, 3] > 0)
        _wind_data = csv_data[_idx]
        self.wind_data = np.concatenate((_wind_data[:, :3], _wind_data[:, 4:7]), axis=1)
        self.kd_tree = KDTree(_wind_data[:, :3])

    def get_wind(self, loc):
        # ANSYS North West Up

        dist, idx = self.kd_tree.query(loc, distance_upper_bound=3)

        wind_east, wind_north, wind_up = 0, 0, 0

        if dist is not np.inf:
            wind_east = -self.wind_data[idx, 4]
            wind_north = self.wind_data[idx, 3]
            wind_up = self.wind_data[idx, 5]

        """
        OpenFoam x:North y:West z:Up
        ANSYS: check frame setting
        return *ENU*
        """

        return wind_east, wind_north, wind_up


"""
could do some interpolation... but not the trilinear with all the values
"""


def main():
    print("This main fn is only for a test")


if __name__ == '__main__':
    main()
