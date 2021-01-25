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

 Modified from mesonh.py
"""

from __future__ import absolute_import, print_function, division
from wind_importer_cfd import CFDImporter
import sys
import signal
import time
# import socket
# import struct
# import cmath
import os
import numpy as np

# Add pprz_home path; this is necessary for pprzlink
PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                      '../../../..')))
if "PAPARAZZI_HOME" not in os.environ:
    os.environ["PAPARAZZI_HOME"] = PPRZ_HOME
sys.path.append(PPRZ_HOME + "/var/lib/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

# M_IN_KM = 1000.

# atm = None


# scale = np.array([1., 1/M_IN_KM, 1/M_IN_KM, 1/M_IN_KM])

class WindDataImporter:
    def __init__(self, args):
        self.origin = np.array([args.origin_x, args.origin_y, args.origin_z])
        self.cfd_importer = CFDImporter()
        self.cfd_importer.init_importer(args.file, args.inlet_wind_speed_east,
                                        args.inlet_wind_speed_north,
                                        args.inlet_wind_speed_up)
        self.wind_default_x = args.inlet_wind_speed_east
        self.wind_default_y = args.inlet_wind_speed_north
        self.wind_default_z = args.inlet_wind_speed_up
        self.wind_prev_x, self.wind_prev_y, self.wind_prev_z = self.wind_default_x, self.wind_default_y, self.wind_default_z
        self.filter_initialized = True

    # a simple low pass filter
    def lpf_simple(self, wind_x, wind_y, wind_z, alpha=0.5):
        # if self.filter_initialized:
        lpf_x = (1-alpha)*wind_x + alpha*self.wind_prev_x
        lpf_y = (1-alpha)*wind_y + alpha*self.wind_prev_y
        lpf_z = (1-alpha)*wind_z + alpha*self.wind_prev_z
        return lpf_x, lpf_y, lpf_z
        # return 0, 0, 0

    def update_estimation(self, wind_x, wind_y, wind_z):
        self.wind_prev_x = wind_x
        self.wind_prev_y = wind_y
        self.wind_prev_z = wind_z

    def initialize_filter(self):
        self.wind_prev_x = self.wind_default_x
        self.wind_prev_y = self.wind_default_y
        self.wind_prev_z = self.wind_default_z

    # this fn takes ~0.3 ms
    def get_wind(self, east, north, up):
        loc = np.array([east, north, up])
        loc = loc - self.origin
        # print("loc:", loc)
        weast, wnorth, wup = self.cfd_importer.get_wind(loc)
        # weast, wnorth, wup = np.random.rand()*3, np.random.rand()*3, np.random.rand()*2
        return weast, wnorth, wup

    def cfd_wind_cb(self, ac_id, msg):
        """
            Callback for paparazzi CFD_WIND requests
            the response should be *ENU*
        """
        # request location (in meters)

        # lat, lon, alt.. & e, n, u (3,4,5)???????????/ ned
        north, east, down = float(msg.get_field(1)), \
                            float(msg.get_field(2)), \
                            float(msg.get_field(3))
        _weast, _wnorth, _wup = self.get_wind(east, north, -down)

        if _weast == _wnorth == _wup == 0:
            self.initialize_filter()
            return

        weast, wnorth, wup = self.lpf_simple(_weast, _wnorth, _wup)
        self.update_estimation(_weast, _wnorth, _wup)

        # east, north, up = float(msg.get_field(3)), \
        #                   float(msg.get_field(4)), \
        #                   float(msg.get_field(5))
        # weast, wnorth, wup = get_wind(east, north, up)
        msg_back = PprzMessage("datalink", "CFD_WIND_DATA")
        msg_back.set_value_by_name("ac_id", msg.get_field(0))
        msg_back.set_value_by_name("wind_east", weast)  # TODO
        msg_back.set_value_by_name("wind_north", wnorth)
        msg_back.set_value_by_name("wind_up", wup)
        # ivy.send_raw_datalink(msg_back)
        ivy.send(msg_back)


def signal_handler(signal, frame):
    print('\nShutting down IVY...')
    ivy.shutdown()
    print("Done.")


def main():
    # parse arguments
    import argparse as ap

    argp = ap.ArgumentParser(description="Wind data provider "
                                         "for Paparazzi from CFD or potential flow simulation")

    argp.add_argument("-f", "--file", required=False,
                      default="/home/sunyou/tud/cfd/result_csv_files/export_hill_rev_100m_12.csv",
                      help="CFD result file in full path")

    argp.add_argument("-t", "--time-step", required=False, type=int,
                      help="Time step for importing dynamic/time-variant CFD simulation. "
                           "Not required for steady-states.")

    argp.add_argument("-x", "--origin-x", required=False, type=float,
                      default=0.,
                      help="Origin position x (EAST).")
    argp.add_argument("-y", "--origin-y", required=False, type=float,
                      default=100.,
                      help="Origin position y (NORTH).")
    argp.add_argument("-z", "--origin-z", required=False, type=float,
                      default=0.,
                      help="Origin position z (UP).")

    # default wind speed in ENU
    argp.add_argument("-i", "--inlet-wind-speed-east", required=False, type=float,
                      default=0,
                      help="Inlet wind speed (EAST), default value is (0, 10, 0) in ENU")
    argp.add_argument("-j", "--inlet-wind-speed-north", required=False, type=float,
                      default=-12,
                      help="Inlet wind speed (NORTH), default value is (0, 10, 0) in ENU")
    argp.add_argument("-k", "--inlet-wind-speed-up", required=False, type=float,
                      default=0.,
                      help="Inlet wind speed (UP), default value is (0, 10, 0) in ENU")

    args = argp.parse_args()

    # register signal handler for ctrl+c to stop the program
    signal.signal(signal.SIGINT, signal_handler)

    importer = WindDataImporter(args)

    # global origin
    # origin = np.array([args.origin_x, args.origin_y, args.origin_z])

    # build atmosphere simulation source
    # global importer
    # if args.type == 0:
    #     importer = CFDImporter()
    #     importer.init_importer(args.file, args.inlet_wind_speed_east, args.inlet_wind_speed_north,
    #                            args.inlet_wind_speed_up)
    # #     elif arguments.type == 1:
    # #         importer = PotentialFlowImporter()
    # else:
    #     print("Please specify importer type")

    global ivy
    ivy = IvyMessagesInterface("WindSimulationData")
    # ivy.subscribe(cfd_wind_cb, '(.* CFD_WIND_REQ .*)')
    ivy.subscribe(importer.cfd_wind_cb, '(.* LTP_POSITION .*)')

    # wait for ivy to stop
    from ivy.std_api import IvyMainLoop  # noqa

    signal.pause()


if __name__ == '__main__':
    main()
