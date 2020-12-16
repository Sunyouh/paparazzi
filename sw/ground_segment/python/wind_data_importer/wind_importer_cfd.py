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
from scipy import interpolate


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

    def init_importer(self, f_name, method="nearest"):
        print("init importer")
        self.read_csv(f_name)

        # self.linear_interpolator = interpolate.RegularGridInterpolator(self.cfd_points, self.cfd_u, "linear")
        # x_arr = (self.cfd_u[0], self.cfd_u[0], self.cfd_u[0])
        # x_arr_2 = np.array(zip(x_arr))
        # self.interpolator_x = interpolate.RegularGridInterpolator(self.cfd_points, x_arr, method=method)
        # self.interpolator_y = interpolate.RegularGridInterpolator(self.cfd_points, self.cfd_u[1], method=method)
        # self.interpolator_z = interpolate.RegularGridInterpolator(self.cfd_points, self.cfd_u[2], method=method)

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

    # paraview csv; P0, P1, P2, P_mag, U0, U1, U2, U_mag, eps, k, nut, p, block, p_id
    def read_csv(self, f_name):
        csv_data = np.genfromtxt(fname=f_name, delimiter=",", skip_header=5)
        # _pts = csv_data[1:, :3]
        # print("pts: ", _pts.shape)
        # _u = csv_data[1:, 4:7]
        # print("u: ", _u.shape)
        self.wind_data = np.concatenate((csv_data[1:, :3], csv_data[1:, 4:7]), axis=1)
        # print(self.wind_data.shape)
        # self.cfd_points = (_pts[:, 0], _pts[:, 1], _pts[:, 2])
        # self.cfd_u = (_u[:, 0], _u[:, 1], _u[:, 2])
        # print(len(self.cfd_points))
        # print(len(self.cfd_u))

    def get_wind(self, loc):
        # loc: (east, north, up) == (y, x, z)
        # self.count += 1
        # print(self.count)
        # print("get_wind ", self.wind_data.shape)
        _x_nearest_arr = self.find_nearest(self.wind_data, loc[1], 0)
        _y_nearest_arr = self.find_nearest(_x_nearest_arr, loc[0], 1)
        _z_nearest_arr = self.find_nearest(_y_nearest_arr, loc[2], 2)
        wind_data = list(_z_nearest_arr)[0]
        """
        OpenFoam x:North y:West z:Up
        ANSYS: ?
        return *ENU*
        """
        # East North Up
        return (wind_data[4], wind_data[3], wind_data[5])

    def find_nearest(self, arr, value, axis):
        # print(value, axis)
        err = np.abs(np.asarray(arr[:, axis]) - value)
        # err_min = err.min()
        # print("min_err: ", err_min)
        idx = np.where(err == err.min())
        return np.array(arr[idx, :])[0]


    # def get_points(self, points, var, method='nearest'):
    #     """ Get value of variable on points
    #
    #     Arguments:
    #     points: a ndarray containing the point coordinates on the last
    #     dimension
    #     var: the name of the variable in the mesoNH file(s)
    #     method: 'nearest' and 'linear' interpolation are currently supported
    #     """
    #     points = np.array(points)
    #     p = self._apply_bounds(points)
    #     caxes = tuple(range(p.ndim - 1))
    #     bounds = zip(p.min(axis=caxes), p.max(axis=caxes))
    #     interpolator = self._get_interpolator(bounds, var, method)
    #     return interpolator(p, method).squeeze()
    #
    # def _get_interpolator(self, bounds, var, method="nearest"):
    #     slice_indexes, coordinates = self._slicyfy(bounds)
    #     values = self._get_var_values(var, slice_indexes)
    #     ip = RegularGridInterpolator(coordinates, values, method)
    #     return ip
    #
    # def _slicyfy(self, bounds):
    #     slice_indexes = ()
    #     coordinates = ()
    #     for d, b in enumerate(bounds):
    #         dslice = slice(find_le(self.grid_coordinates[d], b[0]),
    #                        find_gt(self.grid_coordinates[d], b[1]) + 1)
    #         slice_indexes += dslice,
    #         coordinates += self.grid_coordinates[d][dslice],
    #
    #     return slice_indexes, coordinates
    #
    # def _get_var_values(self, var, idx=Ellipsis):
    #     return self.data[var][idx]


"""
# Create coordinate pairs
cartcoord = list(zip(x, y))


X = np.linspace(min(x), max(x))
Y = np.linspace(min(y), max(y))
X, Y = np.meshgrid(X, Y)

# Approach 1
interp = scipy.interpolate.LinearNDInterpolator(cartcoord, z, fill_value=0)
Z0 = interp(X, Y)
"""


def main():
    print("This main fn is for testing")
    fname = "/home/sunyou/tud/cfd/export_sample.csv"
    importer = CFDImporter()
    importer.init_importer(fname)
    wind = importer.get_wind((123, -50.5, 150))
    print(wind)


if __name__ == '__main__':
    main()