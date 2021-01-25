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
        # self.cfd_points[0, :] += x
        # self.cfd_points[1, :] += y
        # self.cfd_points[2, :] += z
        self.translate_origin += (x, y, z)

    # def interpolate3d(self, x, y, z):
    #     return self.linear_interpolator(np.array((x, y, z)))

    # def interpolate3d(self, _loc):
    #     loc = _loc - self.translate_origin
    #     wind_x = self.interpolator_x(loc)
    #     wind_y = self.interpolator_y(loc)
    #     wind_z = self.interpolator_z(loc)
    #     return wind_x, wind_y, wind_z

    def read_csv(self, f_name):
        csv_data = np.genfromtxt(fname=f_name, delimiter=",", skip_header=5)
        self.wind_data = np.concatenate((csv_data[1:, :3], csv_data[1:, 4:7]), axis=1)

    def get_wind(self, loc):
        # loc: (east, north, up) -> (y, x, z) in NED

        if not (self.x_min < loc[1] < self.x_max and self.y_min < loc[0] < self.y_max and self.z_min < loc[2] < self.z_max):
            return self.inlet_wind_speed_enu

        _x_nearest_arr = self.find_nearest(self.wind_data, loc[1], 0)
        _z_nearest_arr = self.find_nearest(_x_nearest_arr, loc[2], 2)
        _y_nearest_arr = self.find_nearest(_z_nearest_arr, loc[0], 1)
        wind_data = list(_y_nearest_arr)[0]
        """
        OpenFoam x:North y:West z:Up
        ANSYS: check frame setting
        return *ENU*
        """
        if (abs(wind_data[3])+abs(wind_data[4])+abs(wind_data[5])) < self.inlet_wind_speed_sum:
            return (0, 0, 0)
        # East North Up TODO: is this enu????
        return (wind_data[4], wind_data[3], wind_data[5])

    def find_nearest(self, arr, value, axis):
        # print(value, axis)
        err = np.abs(np.asarray(arr[:, axis]) - value)
        # err_min = err.min()
        # print("min_err: ", err_min)
        idx = np.where(err == err.min())
        return np.array(arr[idx, :])[0]


"""
could do some interpolation... but not the trilinear with all the values
maybe linear interp btw two nearest points? but it is 3d - how to determine what is the nearest?
"""


def main():
    print("This main fn is only for a test")
    # fname = "/home/sunyou/tud/cfd/export_sample.csv"
    # importer = CFDImporter()
    # importer.init_importer(fname)
    # sum_t = 0
    #
    # for i in range(500):
    #     _t1 = time.time()
    #     # wind = importer.get_wind((123, -50.5, 150))
    #     wind = importer.get_wind((np.random.randint(0, 300), np.random.randint(-50, 50), np.random.randint(0, 300)))
    #     _t2 = time.time()
    #     sum_t += (_t2 - _t1)*1000
    #
    # print(sum_t / 500)
    # # print(wind)


if __name__ == '__main__':
    main()
